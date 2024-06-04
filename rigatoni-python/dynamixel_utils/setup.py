# Available at setup time due to pyproject.toml
import os

os.environ["CC"] = "gcc"
os.environ["CXX"] = "g++"

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

package_name = "_dynamixel_utils"

# The main interface is through Pybind11Extension.
# * You can add cxx_std=11/14/17, and then build_ext can be removed.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53)

ext_modules = [
    Pybind11Extension(
        f"{package_name}",
        ["source/fixed_frequency_loop_manager.cpp"],
        # Example: passing in the version to the compiled code
        cxx_std=14,
        include_dirs=["include"],
    ),
]

setup(
    name=f"{package_name}",
    author="Cole Ten",
    author_email="cten@ucla.edu",
    description="Helpers for working with dynamixel motors",
    ext_modules=ext_modules,
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=True,
    python_requires=">=3.10",
    setup_requires=["pybind11>=2.6"],
    package_data={
        f"{package_name}": ["py.typed", f"{package_name}.pyi", "MX28-AR-2.json"]
    },
)
