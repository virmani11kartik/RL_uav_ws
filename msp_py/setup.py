from setuptools import setup, find_packages

setup(
    name="msp_monitor",
    version="1.0.0",
    author="uav_0",
    description="MSP serial data monitor and visualizer with PyQt GUI and pyqtgraph plots",
    packages=find_packages(exclude=["tests", "docs"]),
    python_requires=">=3.8",
    install_requires=[
        # GUI & plotting
        "PyQt5>=5.15; platform_machine != 'aarch64'",  # use apt on Pi if build fails
        "PySide6>=6.7; platform_machine == 'aarch64'", # fallback for Pi 5 (ARM)
        "pyqtgraph>=0.13.3",
        
        # Serial comms & utilities
        "pyserial>=3.5",
        "numpy>=1.25",
        "matplotlib>=3.8",

        # Optional: if you’re logging or exporting data
        "pandas>=2.0",
    ],
    entry_points={
        "console_scripts": [
            "msp-monitor = msp_monitor:main",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
    ],
)
