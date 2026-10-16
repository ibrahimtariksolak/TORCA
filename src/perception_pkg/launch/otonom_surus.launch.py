"""Otonom surus: Gazebo + arac + algilama + kontrol.

Onceki surumdeki sorunlar:
  - Yollar '~/ros2_ws/src/perception_pkg' olarak sabitti; colcon install
    anlamsiz hale geliyor, setup.py'daki data_files bosa gidiyordu ve
    paket baska bir makinede calismiyordu.
  - URDF dosyasi .urdf.xacro adini tasidigi halde duz open() ile
    okunuyordu; xacro ifadeleri hic islenmiyordu.
  - Hicbir node'a use_sim_time verilmiyordu. Gazebo sim saati yayinlarken
    node'lar duvar saatinde calisiyor, zaman damgalari tutarsiz kaliyordu.
  - Dugumler TimerAction ile 3/4/5 saniyeye dizilmisti; yavas makinede
    spawn Gazebo hazir olmadan calisiyordu.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (Command, LaunchConfiguration,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('perception_pkg')

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_share, 'worlds', 'otoban.world'),
        description='Yuklenecek Gazebo dunyasi')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='gzclient (Gazebo arayuzu) acilsin mi')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='RViz acilsin mi')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Dugumler Gazebo saatini kullansin')
    # Arac dinamigini (direksiyon, hiz, surtunme) olcerken algilama
    # dugumleri kapatilabilmeli: serit_takip cmd_vel'e yazdigi surece
    # manuel komutlarla yarisir, sign_detector ise uc YOLO ornegi
    # acildiginda Gazebo'nun gercek zaman carpanini yere seriyor.
    algilama_arg = DeclareLaunchArgument(
        'algilama', default_value='true',
        description='sign_detector ve serit_takip calissin mi')
    guvenlik_arg = DeclareLaunchArgument(
        'guvenlik', default_value='true',
        description='LiDAR tabanli acil fren katmani aktif mi')
    # Trafik dugumu yalnizca dunyada trafik_N modelleri varsa anlamli;
    # bos pistte acilirsa set_entity_state cagrilari bos yere doner.
    trafik_arg = DeclareLaunchArgument(
        'trafik', default_value='0',
        description='kac hareketli trafik araci surulsun (dunyada ayni '
                    'sayida trafik_N modeli olmali)')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_yaml = os.path.join(pkg_share, 'config', 'params.yaml')

    urdf_xacro = os.path.join(pkg_share, 'urdf', 'otonom_arac.urdf.xacro')
    # xacro ile isle: dosya gercek xacro, duz okunamaz.
    # ParameterValue(..., value_type=str) sart: aksi halde launch,
    # xacro'nun urettigi XML'i YAML sanip ayristirmaya calisiyor ve
    # "Unable to parse the value of parameter robot_description" ile
    # patliyor.
    robot_desc = ParameterValue(Command(['xacro ', urdf_xacro]),
                                value_type=str)

    # Dunyadaki tabelalar model:// ile cagriliyor. Paketin kendi models/
    # dizinini arama yoluna ekle ki ~/.gazebo/models disinda da bulunsun.
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_share, 'models') + ':' +
        os.path.expanduser('~/.gazebo/models') + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', ''))

    gzserver = ExecuteProcess(
        # libgazebo_ros_state.so BURADA OLMAMALI: o bir WorldPlugin'dir,
        # -s ile verilince "incorrect plugin type" ile sessizce dusuyor.
        # Dunya dosyasinin icinde tanimli (bkz. generate_otoban.py).
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world],
        output='screen')

    gzclient = ExecuteProcess(
        cmd=['gzclient'], output='screen',
        condition=IfCondition(LaunchConfiguration('gui')))

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim_time}])

    # Eski surumde 7 argumanli pozisyonel bicim kullaniliyordu; Humble'da
    # bu bicim kullanimdan kaldirildi ve her acilista uyari basiyordu.
    world_odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_odom_tf', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'odom'])

    # Spawn pozu worlds/generate_otoban.py tarafindan hesaplanir:
    # oval pistin alt duz bolumu, sag seridin ortasi. z, tekerlek
    # yaricapindan (0.33) biraz yuksek olmali ki araç zemine gommesin.
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'otonom_arac',
                   '-x', '-18.0', '-y', '-26.75', '-z', '0.40',
                   '-Y', '0.0'])

    algilama = IfCondition(LaunchConfiguration('algilama'))

    sign_detector = Node(
        package='perception_pkg', executable='sign_detector',
        name='sign_detector', output='screen',
        parameters=[params_yaml, {'use_sim_time': use_sim_time}],
        condition=algilama)

    # Guvenlik aciksa serit_takip cmd_vel_cmd'ye yaziyor ve guvenlik
    # oradan okuyup filtreleyerek cmd_vel'e yayinliyor. Guvenlik
    # kapaliysa remap ile cmd_vel_cmd dogrudan cmd_vel'e gidiyor.
    # Iki node tanimi gerekli cunku LaunchConfiguration cozumleme
    # zamanlamasi sart kosullu remap'e izin vermiyor.
    # algilama & guvenlik -> serit_takip + guvenlik, remap yok
    # algilama & !guvenlik -> serit_takip, remap cmd_vel_cmd -> cmd_vel
    # !algilama -> hicbiri calismiyor
    #
    # launch icin bu ucuncu kosul 'ikili' secimi iki ayri Node tanimina
    # bolmek gerektiriyor; LaunchConfiguration cozumleme zamanlamasi
    # sart kosullu remap'e izin vermiyor.
    from launch.substitutions import AndSubstitution, NotSubstitution

    algilama = LaunchConfiguration('algilama')
    guvenlik = LaunchConfiguration('guvenlik')

    aci_ve_guv = AndSubstitution(algilama, guvenlik)
    aci_sade = AndSubstitution(algilama, NotSubstitution(guvenlik))

    serit_takip_guvenlikli = Node(
        package='perception_pkg', executable='serit_takip',
        name='serit_takip', output='screen',
        parameters=[params_yaml, {'use_sim_time': use_sim_time}],
        condition=IfCondition(aci_ve_guv))
    serit_takip_direkt = Node(
        package='perception_pkg', executable='serit_takip',
        name='serit_takip', output='screen',
        parameters=[params_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/arac/cmd_vel_cmd', '/arac/cmd_vel')],
        condition=IfCondition(aci_sade))

    guvenlik_node = Node(
        package='perception_pkg', executable='guvenlik',
        name='guvenlik', output='screen',
        parameters=[params_yaml, {'use_sim_time': use_sim_time}],
        condition=IfCondition(guvenlik))

    # trafik:=0 (varsayilan) ise dugum hic baslatilmaz
    trafik_node = Node(
        package='perception_pkg', executable='trafik',
        name='trafik', output='screen',
        parameters=[params_yaml,
                    {'use_sim_time': use_sim_time,
                     'arac_sayisi': LaunchConfiguration('trafik')}],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('trafik'), '" != "0"'])))

    rviz_config = os.path.join(pkg_share, 'config', 'otonom.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        world_arg, gui_arg, rviz_arg, sim_time_arg, algilama_arg,
        guvenlik_arg, trafik_arg,
        model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        world_odom_tf,

        # Sabit gecikme yerine olay zinciri: gzserver gercekten
        # basladiktan sonra spawn, spawn bittikten sonra dugumler.
        RegisterEventHandler(OnProcessStart(
            target_action=gzserver, on_start=[spawn])),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[sign_detector, serit_takip_guvenlikli,
                     serit_takip_direkt, guvenlik_node, trafik_node,
                     rviz])),
    ])
