from setuptools import find_packages, setup

setup(
    name="RobotControl",
    version="1.0",
    author="Jianwei Han",
    url="gitlab.com",
    description="Robot python Control",
    packages=find_packages(),
    package_data={
        '': ['*']
    },
    zip_safe=False
)
