from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'mini_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*')), 

            (os.path.join('share', package_name), glob('URDF/*')) 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alakh',
    maintainer_email='alakh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture = mini_project.capture_image:main',
            'line = mini_project.line_follow:main',
            'line1 = mini_project.ball_follower:main',
            'ball = mini_project.line_follow_og:main'
        ],
    },
)
