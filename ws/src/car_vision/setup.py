from setuptools import find_packages, setup

package_name = 'car_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexis',
    maintainer_email='alexischaparomero@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_vision = car_vision.car_vision:main',
            'thresholding = car_vision.thresholding:main',
            'ros_cam_calibration = car_vision.ros_cam_calibration:main',
            'take_image = car_vision.take_image:main',
            'car_inference = car_vision.car_inference:main',
            'webcamPublisher = car_vision.webcamPublisher:main',
            'intersection_detector = car_vision.intersection_detector:main',
            'img_comp = car_vision.img_comp:main'
        ],
    },
)
