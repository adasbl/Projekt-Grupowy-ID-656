import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions
import subprocess
import os
import signal
import time
from datetime import datetime

class ControlPanel(Node):
    def __init__(self):
        super().__init__('control_panel')
        
        # Klienci do sterowania lidarem
        self.start_motor_client = self.create_client(Empty, '/start_motor')
        self.stop_motor_client = self.create_client(Empty, '/stop_motor')
        
        # Publisher do zatrzymywania robota
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Zmienne procesów w tle
        self.slam_process = None
        self.nav2_process = None
        self.explore_process = None
        self.custom_nav_process = None # <-- Dodane dla Twojego skryptu

        # Zmienna przechowująca referencję do aktualnego pliku logów
        self.common_log_file = None

    def call_motor(self, client, nazwa_akcji):
        print("Łączenie z Jetsonem...")
        if not client.wait_for_service(timeout_sec=3.0):
            print("Błąd: Serwis niedostępny! (Czy jetson_core działa na Jetsonie?)")
            return
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        print(f"Polecenie wysłane na Jetsona: {nazwa_akcji}")

    def get_output_target(self):
        """Zwraca plik logów, jeśli istnieje, lub odrzuca logi, jeśli go nie ma."""
        return self.common_log_file if self.common_log_file else subprocess.DEVNULL

    def write_to_log(self, message):
        """Pomocnicza funkcja do zapisywania z datą, jeśli plik logów jest otwarty."""
        if self.common_log_file and not self.common_log_file.closed:
            timestamp = datetime.now().strftime('%H:%M:%S')
            self.common_log_file.write(f"\n[{timestamp}] {message}\n")
            self.common_log_file.flush()

    def start_slam(self):
        if self.slam_process is not None and self.slam_process.poll() is None:
            print("System SLAM już działa!")
            return
            
        print(">>> Uruchamiam system SLAM i RViz w tle...")
        self.write_to_log("--- URUCHAMIANIE SLAM ---")
        
        current_dir = os.path.dirname(os.path.realpath(__file__))
        slam_launch_file = os.path.join(current_dir, 'pc_slam.launch.py')
        
        self.slam_process = subprocess.Popen(
            ["ros2", "launch", slam_launch_file], 
            preexec_fn=os.setsid,
            stdout=self.get_output_target(),
            stderr=subprocess.STDOUT
        )

    def start_nav2(self):
        if self.nav2_process is not None and self.nav2_process.poll() is None:
            return
            
        print(">>> Uruchamiam system Nav2 w tle...")
        self.write_to_log("--- URUCHAMIANIE NAV2 ---")
        
        current_dir = os.path.dirname(os.path.realpath(__file__))
        nav2_launch_file = os.path.join(current_dir, 'pc_nav2.launch.py')
        
        self.nav2_process = subprocess.Popen(
            ["ros2", "launch", nav2_launch_file], 
            preexec_fn=os.setsid,
            stdout=self.get_output_target(),
            stderr=subprocess.STDOUT
        )

    def start_exploration(self):
        if self.explore_process is not None and self.explore_process.poll() is None:
            return
            
        print(">>> Uruchamiam system Explore Lite (Autonomiczna Eksploracja)...")
        self.write_to_log("--- URUCHAMIANIE M-EXPLORE ---")
        
        self.explore_process = subprocess.Popen(
            ["ros2", "launch", "explore_lite", "explore.launch.py"], 
            preexec_fn=os.setsid,
            stdout=self.get_output_target(),
            stderr=subprocess.STDOUT
        )

    def stop_slam(self):
        print(">>> Zatrzymuję system SLAM...")
        if self.slam_process:
            try:
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGINT)
                self.slam_process.wait(timeout=5.0)
            except Exception:
                pass
            self.slam_process = None
                
        subprocess.run(["pkill", "-9", "-f", "slam_toolbox"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "rf2o_laser_odometry"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "rviz2"], stderr=subprocess.DEVNULL)

    def stop_nav_systems(self):
        print(">>> Zatrzymuję systemy Nawigacji i Eksploracji...")
        
        # Zatrzymywanie Nav2
        if self.nav2_process:
            try:
                os.killpg(os.getpgid(self.nav2_process.pid), signal.SIGINT)
                self.nav2_process.wait(timeout=5.0)
            except Exception:
                pass
            self.nav2_process = None
                
        # Zatrzymywanie Explore Lite
        if self.explore_process:
            try:
                os.killpg(os.getpgid(self.explore_process.pid), signal.SIGINT)
                self.explore_process.wait(timeout=5.0)
            except Exception:
                pass
            self.explore_process = None

        # Zatrzymywanie własnego skryptu nawigacji
        if self.custom_nav_process:
            try:
                os.killpg(os.getpgid(self.custom_nav_process.pid), signal.SIGINT)
                self.custom_nav_process.wait(timeout=5.0)
            except Exception:
                pass
            self.custom_nav_process = None

        subprocess.run(["pkill", "-9", "-f", "component_container"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "explore"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "robot_navigation.py"], stderr=subprocess.DEVNULL)
        
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        print("[PANEL] Wysłano sygnał stopu (Prędkość wyzerowana).")
    
    def shutdown_system(self):
        print("\n[ZAMYKANIE] Czyszczenie procesów na PC...")
        self.stop_nav_systems()
        self.stop_slam()
        self.stop_robot()
        subprocess.run(["pkill", "-9", "-f", "teleop_twist_keyboard"], stderr=subprocess.DEVNULL)
        
        # Bezpieczne zamknięcie pliku, jeśli był otwarty
        if self.common_log_file and not self.common_log_file.closed:
            self.write_to_log("=== KONIEC SESJI PANELU ===")
            self.common_log_file.close()
            
        print("Gotowe. Do widzenia!")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    panel = ControlPanel()

    try:
        print("\n[INICJALIZACJA] Czyszczenie środowiska...")
        panel.stop_nav_systems()
        panel.stop_slam() 
        subprocess.run(["pkill", "-9", "-f", "teleop_twist_keyboard"], stderr=subprocess.DEVNULL)
        print("[INICJALIZACJA] Panel PC gotowy do pracy.\n")

        while rclpy.ok():
            print("="*55)
            print("          PANEL STEROWANIA (PC) - NAV2 & CUSTOM")
            print("="*55)
            print("1 - Włącz silnik lidaru (na Jetsonie)")
            print("2 - Wyłącz silnik lidaru (na Jetsonie)")
            print("3 - Włącz budowanie mapy (SLAM + RViz)")
            print("4 - Zatrzymanie mapowania i nawigacji")
            print("5 - Sterowanie ręczne (Teleop)")
            print("6 - Autonomiczna eksploracja (Nav2 + M-Explore)")
            print("7 - Autonomiczna eksploracja (Własny algorytm DWA/A*)")
            print("0 - Wyjście")
            print("="*55)
            
            c = input("Wybierz akcję (0-7): ")
            
            if c == '1': 
                panel.call_motor(panel.start_motor_client, "Włączono silnik lidaru")
            elif c == '2': 
                panel.call_motor(panel.stop_motor_client, "Zatrzymano silnik lidaru")
            elif c == '3': 
                panel.start_slam()
            elif c == '4': 
                panel.stop_nav_systems()
                panel.stop_slam()
            elif c == '5':
                print("\n>>> Uruchamiam klawiaturę... (Wciśnij Ctrl+C, aby wrócić do menu) <<<")
                subprocess.run(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
            
            elif c == '6' or c == '7':
                nazwa_trybu = "Nav2" if c == '6' else "Własny Algorytm"
                print(f"\n>>> Uruchamiam sekwencję autonomicznej nawigacji ({nazwa_trybu})...")
                
                # --- 1. TWORZENIE FOLDERU NA MAPY I LOGI PRZED STARTEM ---
                current_dir = os.path.realpath(os.path.dirname(__file__))
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                logs_dir = os.path.join(current_dir, 'logs')
                run_folder = os.path.join(logs_dir, f"run_{timestamp}")
                os.makedirs(run_folder, exist_ok=True)
                
                # Przekazanie ścieżki (dla nav_debug.txt w trybie 7)
                env_vars = os.environ.copy()
                env_vars["ROBOT_RUN_DIR"] = run_folder

                # Otwarcie pliku na logi systemowe W TYM FOLDERZE
                log_file_path = os.path.join(run_folder, 'system_log.txt')
                panel.common_log_file = open(log_file_path, 'w')
                panel.write_to_log(f"=== START SESJI AUTO-EKSPLORACJI: {nazwa_trybu} ({timestamp}) ===")

                # --- 2. URUCHAMIANIE SYSTEMÓW ---
                panel.start_slam()
                print("Czekam 3 sekundy na zainicjalizowanie SLAM...")
                time.sleep(3.0)
                
                if c == '6':
                    # Tryb 6: Oficjalne Nav2 + Explore Lite
                    panel.start_nav2()
                    print("Czekam 6 sekund na aktywację serwerów i map kosztów Nav2...")
                    time.sleep(6.0)
                    panel.start_exploration()
                else:
                    # Tryb 7: Twój skrypt robot_navigation.py
                    print(">>> Uruchamiam Twój skrypt nawigacji (robot_navigation.py)...")
                    panel.write_to_log("--- URUCHAMIANIE WŁASNEGO ALGORYTMU DWA/A* ---")
                    nav_script = os.path.join(current_dir, 'robot_navigation.py')
                    panel.custom_nav_process = subprocess.Popen(
                        ["python3", nav_script], 
                        env=env_vars,
                        preexec_fn=os.setsid,
                        stdout=panel.get_output_target(),
                        stderr=subprocess.STDOUT
                    )
                
                print(f"\n>>> Robot rozpoczął eksplorację! <<<")
                print(f"Podgląd logów głównych: {log_file_path}")
                if c == '7':
                    print(f"Logi debuggowania (DWA): {os.path.join(run_folder, 'nav_log.txt')}")
                print("Wciśnij Ctrl+C, aby zatrzymać robota i zapisać mapę.")
                
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\nZatrzymano proces nawigacji.")
                finally:
                    # Zamknięcie pliku system_log.txt
                    if panel.common_log_file and not panel.common_log_file.closed:
                        panel.write_to_log("=== ZATRZYMANO AUTO-EKSPLORACJĘ ===")
                        panel.common_log_file.close()
                        panel.common_log_file = None
                    
                    panel.stop_nav_systems()
                    panel.stop_robot()
                    
                    print("\n>>> Trwa zapisywanie końcowej mapy...")
                    try:
                        map_file_path = os.path.join(run_folder, "mapa")
                        subprocess.run([
                            "ros2", "run", "nav2_map_server", "map_saver_cli", 
                            "-f", map_file_path
                        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        
                        print(f"[PANEL] Sukces! Mapa została zapisana w: {run_folder}")
                        
                        try:
                            from PIL import Image
                            pgm_file = map_file_path + ".pgm"
                            png_file = map_file_path + ".png"
                            if os.path.exists(pgm_file):
                                with Image.open(pgm_file) as img:
                                    img.save(png_file)
                        except ImportError:
                            pass
                    except Exception as e:
                        print(f"[PANEL] Błąd podczas zapisu mapy: {e}")
            elif c == '0':
                break
                
    except KeyboardInterrupt:
        print("\nPrzerwano awaryjnie (Ctrl+C).")
        
    finally:
        panel.shutdown_system()
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
