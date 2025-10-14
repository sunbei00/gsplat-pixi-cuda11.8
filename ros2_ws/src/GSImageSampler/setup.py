from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'GSImageSampler'

def package_files(directory):
    paths = []
    if os.path.isdir(directory):
        for (path, _dirs, filenames) in os.walk(directory):
            for filename in filenames:
                paths.append(os.path.join(path, filename))
    return paths

# 이 폴더들은 setup.py와 같은 디렉터리(패키지 루트)에 있어야 합니다.
config_files = package_files('config')   # 예: config/back_calib.json 등
launch_files = package_files('launch')   # 선택

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='sunbei00@naver.com',
    description='Image sampling + COLMAP dump + CameraInfo publisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sampler = GSImageSampler.sampler:main',
            'kimm_sampler = GSImageSampler.kimm_sampler:main',
        ],
    },
)

