from setuptools import setup

package_name = 'obj_extractor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsv',
    maintainer_email='tsv@tsv.ca',
    description='Package that uses a pre-built torch vision model to extract object bounding boxes',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obj_extractor = obj_extractor.image_subscriber:main',
            'test_obj_extractor = obj_extractor.test_image_sub:main'
        ],
    },
)
