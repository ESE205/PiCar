import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt", "r") as req:
    requirements = req.readlines()

setuptools.setup(
    name="picar",
    version="0.2.0",
    author="Ethan Shry",
    author_email="ehshry@gmail.com",
    description="Platform to interface with Adeept Mars Rover for ESE 205",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ESE205/PiCar",
    packages=["picar"],
    install_requires=requirements,
)
