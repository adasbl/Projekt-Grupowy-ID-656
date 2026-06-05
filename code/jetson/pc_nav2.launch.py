import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Pobieramy ścieżkę do domyślnego pakietu bringup z systemu
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Ścieżka do Twojego folderu z kodem (tam gdzie leży nowo utworzony nav2_params.yaml)
    # Zakładam obecną strukturę, w razie potrzeby dostosuj tę ścieżkę
    current_dir = os.path.dirname(os.path.realpath(__file__))
    params_file_path = os.path.join(current_dir, 'nav2_params.yaml')

    # Deklaracja argumentów uruchomienia
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=params_file_path)

    # Dołączenie oficjalnego pliku launch odpowiedzialnego za nawigację
    # Uruchomi on serwer planera, kontrolera, drzew zachowań oraz serwer odzyskiwania (recoveries)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Użyj czasu symulacji, jeśli działasz w Gazebo'),
            
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file_path,
            description='Pełna ścieżka do pliku parametrów Nav2'),
            
        # Uruchomienie całego stosu nawigacji
        navigation_launch
    ])