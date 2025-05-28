from setuptools import setup
import glob
import os

package_name = 'control_system'
msg_files = glob.glob(os.path.join('msg', '*.msg'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', msg_files),
    ],
    install_requires=['setuptools', 'filterpy', 'scikit-image', 'lap'],
    zip_safe=True,
    maintainer='myeolinmalchi',
    maintainer_email='rkd227@pusan.ac.kr',
    description='사용자 맞춤형 자동 조정 모니터 - 제어 시스템',
    license='MIT',
    entry_points={
        'console_scripts': [
            'face_detection_node = control_system.nodes.face_detection_node:main',
            'face_tracking_node = control_system.nodes.face_tracking_node:main',
            'face_focusing_node = control_system.nodes.face_focusing_node:main',
            'mcu_bridge_node = control_system.nodes.mcu_bridge_node:main',
            'post_process_node = control_system.nodes.post_process_node:main',
            'desktop_bridge_node = control_system.nodes.desktop_bridge_node:main',
        ],
    },
)
