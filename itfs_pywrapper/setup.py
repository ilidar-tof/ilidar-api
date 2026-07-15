from __future__ import annotations

import shutil
import sysconfig
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


__version__ = "2.0.0"

ROOT = Path(__file__).resolve().parent
SDK_ROOT = ROOT.parent
EXT_SUFFIX = sysconfig.get_config_var("EXT_SUFFIX") or ".so"


def pybind11_include_dirs() -> list[str]:
    """Find pybind11 and Python headers without requiring a wheel install."""
    includes: list[str] = []
    try:
        import pybind11  # type: ignore

        includes.append(pybind11.get_include())
    except Exception:
        pass

    for candidate in ("/usr/include", "/usr/local/include"):
        if (Path(candidate) / "pybind11" / "pybind11.h").exists():
            includes.append(candidate)

    python_include = sysconfig.get_paths().get("include")
    if python_include:
        includes.append(python_include)

    return list(dict.fromkeys(includes))


class BuildExt(build_ext):
    """Apply the C++17 flags required by the native iTFS extension."""
    c_opts = {
        "msvc": ["/std:c++17", "/EHsc"],
        "unix": ["-std=c++17", "-O3"],
    }
    l_opts = {
        "msvc": ["ws2_32.lib"],
        "unix": ["-pthread"],
    }

    def build_extensions(self) -> None:
        """Add compiler-specific flags before setuptools builds each target."""
        compiler_type = self.compiler.compiler_type
        for ext in self.extensions:
            ext.extra_compile_args.extend(self.c_opts.get(compiler_type, []))
            ext.extra_link_args.extend(self.l_opts.get(compiler_type, []))
        super().build_extensions()

    def copy_extensions_to_source(self) -> None:
        """Keep normal inplace behavior and verify the package receives the module."""
        super().copy_extensions_to_source()
        self._copy_built_extension_to_package()

    def _copy_built_extension_to_package(self) -> None:
        """Copy a generated extension from build/ when setuptools leaves it there."""
        target_dir = ROOT / "ilidar"
        target_dir.mkdir(parents=True, exist_ok=True)

        candidates = [
            path
            for path in (ROOT / "build").rglob("_itfs*")
            if "ilidar" in path.parts
            and path.name.endswith(EXT_SUFFIX)
            and not path.name.startswith("_itfs_lite")
        ]
        if not candidates:
            candidates = [
                path
                for path in (ROOT / "build").rglob("_itfs*")
                if "ilidar" in path.parts and not path.name.startswith("_itfs_lite")
            ]

        for built_ext in candidates:
            dst = target_dir / built_ext.name
            shutil.copy2(built_ext, dst)
            print(f"copied {built_ext} -> {dst}")


ext_modules = [
    Extension(
        "ilidar._itfs",
        sources=[
            str(SDK_ROOT / "src" / "ilidar_pybind.cpp"),
            str(SDK_ROOT / "src" / "ilidar.cpp"),
        ],
        include_dirs=pybind11_include_dirs() + [str(SDK_ROOT / "src")],
        language="c++",
    ),
]


setup(
    packages=["ilidar"],
    cmdclass={"build_ext": BuildExt},
    ext_modules=ext_modules,
)
