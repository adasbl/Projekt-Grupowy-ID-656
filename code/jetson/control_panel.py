import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist  # <-- DODANY IMPORT
from rclpy.signals import SignalHandlerOptions
import subprocess
import os
import signal
import time

class ControlPanel(Node):
    def __init__(self):
        super().__init__('control_panel')
        
        # Klienty do sterowania lidarem na Jetsonie (komunikacja przez Wi-Fi)
        self.start_motor_client = self.create_client(Empty, '/start_motor')
        self.stop_motor_client = self.create_client(Empty, '/stop_motor')
        
        # Publisher do zatrzymywania robota z poziomu panelu <-- DODANE
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Zmienne do śledzenia procesów odpalanych na PC
        self.slam_process = None

    def call_motor(self, client, nazwa_akcji):
        print("Łączenie z Jetsonem...")
        if not client.wait_for_service(timeout_sec=3.0):
            print("Błąd: Serwis niedostępny! (Czy jetson_core.launch.py działa na Jetsonie?)")
            return
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        print(f"Polecenie wysłane na Jetsona: {nazwa_akcji}")

    def start_slam(self):
        if self.slam_process is not None and self.slam_process.poll() is None:
            print("System SLAM już działa!")
            return
            
        print(">>> Uruchamiam system SLAM i RViz w tle...")
        print(">>> (Logi z tego procesu znajdziesz w pliku 'slam_log.txt')")
        current_dir = os.path.dirname(os.path.realpath(__file__))
        slam_launch_file = os.path.join(current_dir, 'pc_slam.launch.py')
        
        # Tworzymy plik na logi, żeby oczyścić terminal
        self.log_file = open(os.path.join(current_dir, 'slam_log.txt'), 'w')
        
        # Odpalamy proces z przekierowaniem wyjścia do pliku
        self.slam_process = subprocess.Popen(
            ["ros2", "launch", slam_launch_file], 
            preexec_fn=os.setsid,
            stdout=self.log_file,
            stderr=subprocess.STDOUT # Błędy też lecą do pliku
        )

    def stop_slam(self):
        print(">>> Zatrzymuję system SLAM...")
        if rclpy.ok():
            try:
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGINT)
                self.slam_process.wait(timeout=5.0)
            except Exception as e:
                pass
            self.slam_process = None
            
            # Zamykamy plik z logami
            if hasattr(self, 'log_file') and not self.log_file.closed:
                self.log_file.close()
                
        subprocess.run(["pkill", "-9", "-f", "slam_toolbox"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "rf2o_laser_odometry"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "rviz2"], stderr=subprocess.DEVNULL)
        print("SLAM całkowicie wyłączony.")
        
    def stop_robot(self):
        """Wysyła zerowe prędkości, aby natychmiast zatrzymać robota"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        print("[PANEL] Wysłano sygnał stopu (Prędkość wyzerowana).")
    
    
    def shutdown_system(self):
        """Zamykanie wszystkiego przed wyjściem"""
        print("\n[ZAMYKANIE] Czyszczenie procesów na PC...")
        self.stop_slam()
        self.stop_robot()
        subprocess.run(["pkill", "-9", "-f", "teleop_twist_keyboard"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "robot_navigation_node.py"], stderr=subprocess.DEVNULL)
        print("Gotowe. Do widzenia!")

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    panel = ControlPanel()

    try:
        print("\n[INICJALIZACJA] Sprawdzanie stanu systemu...")
        panel.stop_slam() # Upewniamy się, że nie ma starych procesów
        subprocess.run(["pkill", "-9", "-f", "teleop_twist_keyboard"], stderr=subprocess.DEVNULL)
        print("[INICJALIZACJA] Panel PC gotowy do pracy.\n")

        while rclpy.ok():
            print("="*45)
            print("          PANEL STEROWANIA (PC)")
            print("="*45)
            print("1 - Włącz silnik lidaru (na Jetsonie)")
            print("2 - Wyłącz silnik lidaru (na Jetsonie)")
            print("3 - Włącz budowanie mapy (SLAM + RViz)")
            print("4 - Zatrzymanie mapowania")
            print("5 - Sterowanie ręczne (Teleop)")
            print("6 - Autonomiczna eksploracja (Auto-Drive)")
            print("0 - Wyjście")
            print("="*45)
            
            c = input("Wybierz akcję (0-6): ")
            
            if c == '1': 
                panel.call_motor(panel.start_motor_client, "Włączono silnik lidaru")
            elif c == '2': 
                panel.call_motor(panel.stop_motor_client, "Zatrzymano silnik lidaru")
            elif c == '3': 
                panel.start_slam()
            elif c == '4': 
                panel.stop_slam()
            elif c == '5':
                print("\n>>> Uruchamiam klawiaturę... (Wciśnij Ctrl+C, aby wrócić do menu) <<<")
                subprocess.run(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
            elif c == '6':
                print("\n>>> Uruchamiam automatyczne mapowanie (SLAM) i nawigację...")
                
                # 1. Automatyczne odpalenie SLAM + RViz
                panel.start_slam()
                
                print("Czekam 3 sekundy na zainicjalizowanie SLAM i RViz...")
                time.sleep(3.0)
                
                print("\n>>> Uruchamiam autonomiczną nawigację... (Wciśnij Ctrl+C, aby wrócić) <<<")
                try:
                    current_dir = os.path.realpath(os.path.dirname(__file__))
                    nav_script = os.path.join(current_dir, 'robot_navigation.py') 
                    subprocess.run(["python3", nav_script])
                except KeyboardInterrupt:
                    print("\nZatrzymano proces nawigacji.")
                finally:
                    # 1. ZEROWANIE PRĘDKOŚCI przy wyjściu
                    panel.stop_robot()
                    
                    # 2. ZAPIS MAPY DO OSOBNEGO PODFOLDERU <-- ZMIENIONE
                    print("\n>>> Trwa zapisywanie końcowej mapy...")
                    try:
                        # Generowanie znacznika czasu dla nazwy folderu
                        timestamp = time.strftime("%Y%m%d_%H%M%S")
                        
                        # Główna ścieżka do images oraz ścieżka do unikalnego podfolderu dla tego przejazdu
                        images_dir = os.path.join(current_dir, 'images')
                        run_folder = os.path.join(images_dir, f"run_{timestamp}")
                        
                        # Tworzenie całego drzewa folderów (np. images/run_20261104_153022)
                        os.makedirs(run_folder, exist_ok=True)  
                        
                        # Zapis plików pod nazwą "mapa" wewnątrz nowo utworzonego podfolderu
                        map_file_path = os.path.join(run_folder, "mapa")
                        
                        # Wywołanie komendy ROS2 map_saver_cli
                        subprocess.run([
                            "ros2", "run", "nav2_map_server", "map_saver_cli", 
                            "-f", map_file_path
                        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        
                        print(f"[PANEL] Sukces! Mapa i pliki konfiguracyjne zostały zapisane w: {run_folder}")
                        
                        # (Opcjonalnie) Konwersja PGM do standardowego PNG
                        try:
                            from PIL import Image
                            pgm_file = map_file_path + ".pgm"
                            png_file = map_file_path + ".png"
                            if os.path.exists(pgm_file):
                                with Image.open(pgm_file) as img:
                                    img.save(png_file)
                                print(f"[PANEL] Wygenerowano również czytelny plik formatu .png")
                        except ImportError:
                            pass # Ignoruj jeśli biblioteka Pillow nie jest zainstalowana
                            
                    except Exception as e:
                        print(f"[PANEL] Wystąpił błąd podczas próby zapisu mapy: {e}")
                
    except KeyboardInterrupt:
        print("\nPrzerwano awaryjnie (Ctrl+C).")
        panel.shutdown_system()
        
    finally:
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
