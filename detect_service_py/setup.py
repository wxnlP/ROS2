from setuptools import find_packages, setup

package_name = 'detect_service_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/resource", ['resource/pic1.jpg', 'resource/pic2.jpg'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='2661006892@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "detec_py = detect_service_py.detect_demo:main",
            "face_detect_service = detect_service_py.detect_service:main",
            "face_detect_client = detect_service_py.detect_client:main"
        ],
    },
)
