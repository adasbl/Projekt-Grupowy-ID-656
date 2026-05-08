import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
import subprocess

class ControlPanel(Node):
    def __init__(self):
        super().__init__('control_panel')
        self.slam_client = self.create_client(SetBool, '/manage_slam')
        self.start_motor_client = self.create_client(Empty, '/start_motor')
        self.stop_motor_client = self.create_client(Empty, '/stop_motor')

    def call_motor(self, client, nazwa_akcji):
        if not client.wait_for_service(timeout_sec=1.0):
            print("Błąd: Serwis niedostępny! (Czy SLAM jest włączony?)")
            return
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        print(f"Polecenie wysłane: {nazwa_akcji}")

    def call_slam(self, state, ciche_wywolanie=False):
        if not self.slam_client.wait_for_service(timeout_sec=1.0):
            if not ciche_wywolanie:
                print("Błąd: Serwis SLAM niedostępny! (Czy menedżer na Jetsonie działa?)")
            return
        req = SetBool.Request()
        req.data = state
        future = self.slam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not ciche_wywolanie:
            print(f"Menedżer: {future.result().message}")

    def shutdown_system(self):
        """Dedykowana procedura zamykająca wszystkie powiązane procesy"""
        print("\n[ZAMYKANIE] Trwa wyłączanie aktywnych węzłów i czyszczenie procesów...")
        
        # 1. Twarde wyłączenie procesów na PC
        subprocess.run(["pkill", "-9", "-f", "rviz2"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "teleop_twist_keyboard"], stderr=subprocess.DEVNULL)
        
        # 2. Bezpieczne zatrzymanie systemu na Jetsonie
        self.call_slam(False, ciche_wywolanie=True)
        
        print("Wszystkie procesy (RViz, Teleop, SLAM) zostały zamknięte.")

def main(args=None):
    rclpy.init(args=args)
    panel = ControlPanel()

    try:
        print("\n[INICJALIZACJA] Sprawdzanie stanu systemu...")
        subprocess.run(["pkill", "-9", "-f", "teleop_twist_keyboard"], stderr=subprocess.DEVNULL)
        panel.call_slam(False, ciche_wywolanie=True)
        print("[INICJALIZACJA] System gotowy do pracy.\n")

        while rclpy.ok():
            print("="*30)
            print("       PANEL STEROWANIA")
            print("="*30)
            print("1 - Włącz silnik lidaru")
            print("2 - Wyłącz silnik lidaru")
            print("3 - Włącz cały system SLAM")
            print("4 - Wyłącz cały system SLAM")
            print("5 - Sterowanie klawiaturą (Teleop)")
            print("0 - Wyjście i zamknięcie wszystkiego")
            print("="*30)
            
            c = input("Wybierz akcję (0-5): ")
            
            if c == '1': 
                panel.call_motor(panel.start_motor_client, "Włączono silnik")
            elif c == '2': 
                panel.call_motor(panel.stop_motor_client, "Zatrzymano silnik")
            elif c == '3': 
                panel.call_slam(True)
            elif c == '4': 
                panel.call_slam(False)
            elif c == '5':
                print("\n>>> Uruchamiam klawiaturę... (Wciśnij Ctrl+C, aby wrócić do menu) <<<")
                subprocess.run(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
            elif c == '0':
                # Wywołanie nowej funkcji czyszczącej przy wyborze 0
                panel.shutdown_system()
                break
            else:
                print("Nieznana opcja!")
                
    except KeyboardInterrupt:
        print("\nPrzerwano (Ctrl+C).")
        panel.shutdown_system()
        
    finally:
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Panel główny zamknięty.")

if __name__ == '__main__':
    main()
