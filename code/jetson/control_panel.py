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

        # --- WSPÓLNY SYSTEM LOGÓW ---
        current_dir = os.path.dirname(os.path.realpath(__file__))
        self.log_file_path = os.path.join(current_dir, 'system_log.txt')
        
        # Tryb 'w' (write) czyści stary plik przy każdym nowym uruchomieniu panelu.
        self.common_log_file = open(self.log_file_path, 'w')
        
        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.common_log_file.write(f"=== START SESJI PANELU STEROWANIA ({start_time}) ===\n")
        self.common_log_file.flush()

    def call_motor(self, client, nazwa_akcji):
        print("Łączenie z Jetsonem...")
        if not client.wait_for_service(timeout_sec=3.0):
            print("Błąd: Serwis niedostępny! (Czy jetson_core działa na Jetsonie?)")
            return
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        print(f"Polecenie wysłane na Jetsona: {nazwa_akcji}")

    def start_slam(self):
        if self.slam_process is not None and self.slam_process.poll() is None:
            print("System SLAM już działa!")
            return
            
        print(">>> Uruchamiam system SLAM i RViz w tle...")
        self.common_log_file.write(f"\n[{datetime.now().strftime('%H:%M:%S')}] --- URUCHAMIANIE SLAM ---\n")
        self.common_log_file.flush()
        
        current_dir = os.path.dirname(os.path.realpath(__file__))
        slam_launch_file = os.path.join(current_dir, 'pc_slam.launch.py')
        
        self.slam_process = subprocess.Popen(
            ["ros2", "launch", slam_launch_file], 
            preexec_fn=os.setsid,
            stdout=self.common_log_file,      # <--- Wskazanie na wspólny plik
            stderr=subprocess.STDOUT
        )

    def start_nav2(self):
        if self.nav2_process is not None and self.nav2_process.poll() is None:
            return
            
        print(">>> Uruchamiam system Nav2 w tle...")
        self.common_log_file.write(f"\n[{datetime.now().strftime('%H:%M:%S')}] --- URUCHAMIANIE NAV2 ---\n")
        self.common_log_file.flush()
        
        current_dir = os.path.dirname(os.path.realpath(__file__))
        nav2_launch_file = os.path.join(current_dir, 'pc_nav2.launch.py')
        
        self.nav2_process = subprocess.Popen(
            ["ros2", "launch", nav2_launch_file], 
            preexec_fn=os.setsid,
            stdout=self.common_log_file,      # <--- Wskazanie na wspólny plik
            stderr=subprocess.STDOUT
        )

    def start_exploration(self):
        if self.explore_process is not None and self.explore_process.poll() is None:
            return
            
        print(">>> Uruchamiam system Explore Lite (Autonomiczna Eksploracja)...")
        self.common_log_file.write(f"\n[{datetime.now().strftime('%H:%M:%S')}] --- URUCHAMIANIE M-EXPLORE ---\n")
        self.common_log_file.flush()
        
        self.explore_process = subprocess.Popen(
            ["ros2", "launch", "explore_lite", "explore.launch.py"], 
            preexec_fn=os.setsid,
            stdout=self.common_log_file,      # <--- Wskazanie na wspólny plik
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
        if self.nav2_process:
            try:
                os.killpg(os.getpgid(self.nav2_process.pid), signal.SIGINT)
                self.nav2_process.wait(timeout=5.0)
            except Exception:
                pass
            self.nav2_process = None
                
        if self.explore_process:
            try:
                os.killpg(os.getpgid(self.explore_process.pid), signal.SIGINT)
                self.explore_process.wait(timeout=5.0)
            except Exception:
                pass
            self.explore_process = None

        subprocess.run(["pkill", "-9", "-f", "component_container"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "explore"], stderr=subprocess.DEVNULL)
        
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
        
        # Zamykamy wspólny plik logów dopiero na samym końcu
        if hasattr(self, 'common_log_file') and not self.common_log_file.closed:
            end_time = datetime.now().strftime("%H:%M:%S")
            self.common_log_file.write(f"\n=== KONIEC SESJI PANELU ({end_time}) ===\n")
            self.common_log_file.close()
            
        print("Gotowe. Do widzenia!")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    panel = ControlPanel()

    try:
        print(f"\n[INICJALIZACJA] Logi zapisywane do: {panel.log_file_path}")
        panel.stop_nav_systems()
        panel.stop_slam() 
        subprocess.run(["pkill", "-9", "-f", "teleop_twist_keyboard"], stderr=subprocess.DEVNULL)
        print("[INICJALIZACJA] Panel PC gotowy do pracy.\n")

        while rclpy.ok():
            print("="*45)
            print("          PANEL STEROWANIA (PC) - NAV2")
            print("="*45)
            print("1 - Włącz silnik lidaru (na Jetsonie)")
            print("2 - Wyłącz silnik lidaru (na Jetsonie)")
            print("3 - Włącz budowanie mapy (SLAM + RViz)")
            print("4 - Zatrzymanie mapowania i nawigacji")
            print("5 - Sterowanie ręczne (Teleop)")
            print("6 - Autonomiczna eksploracja (Nav2 + M-Explore)")
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
                panel.stop_nav_systems()
                panel.stop_slam()
            elif c == '5':
                print("\n>>> Uruchamiam klawiaturę... (Wciśnij Ctrl+C, aby wrócić do menu) <<<")
                subprocess.run(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
            elif c == '6':
                print("\n>>> Uruchamiam sekwencję autonomicznej nawigacji...")
                
                current_dir = os.path.realpath(os.path.dirname(__file__))
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                images_dir = os.path.join(current_dir, 'logs')
                run_folder = os.path.join(images_dir, f"run_{timestamp}")
                os.makedirs(run_folder, exist_ok=True)

                panel.start_slam()
                print("Czekam 3 sekundy na zainicjalizowanie SLAM...")
                time.sleep(3.0)
                
                panel.start_nav2()
                print("Czekam 6 sekund na aktywację serwerów i map kosztów Nav2...")
                time.sleep(6.0)

                panel.start_exploration()
                
                print(f"\n>>> Robot rozpoczął eksplorację! (Podgląd logów: {panel.log_file_path}) <<<")
                print("Wciśnij Ctrl+C, aby zatrzymać robota i zapisać mapę.")
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\nZatrzymano proces nawigacji.")
                finally:
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