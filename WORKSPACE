# -*- python -*-

workspace(name = "drake_franka_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "3e86d88df36f1a7d34cb5663e5f404eb43ce4e42",
    "c6fdd23130b08221ec5343891df99c3ced394fe3fdc7306fc77cf3e535f13004",
)

# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Download a specific commit of Drake, from github.
http_archive(
    name = "drake",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT),
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

load("@drake//tools/workspace/eigen:repository.bzl", "eigen_repository")
eigen_repository(name = "eigen")

load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("@drake//tools/workspace/rules_python:repository.bzl", "rules_python_repository")  # noqa
rules_python_repository(name = "rules_python", mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/cc:repository.bzl", "cc_repository")
cc_repository(name = "cc")

load("@drake//tools/workspace/fmt:repository.bzl", "fmt_repository")
fmt_repository(name = "fmt", mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/gflags:repository.bzl", "gflags_repository")
gflags_repository(name = "gflags")

load("@drake//tools/workspace/glib:repository.bzl", "glib_repository")
glib_repository(name = "glib")

load("@drake//tools/workspace:github.bzl", "github_archive")

load("@drake//tools/workspace/lcm:repository.bzl", "lcm_repository")
lcm_repository(name = "lcm", mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/python:repository.bzl", "python_repository")
python_repository(name = "python")

load("@drake//tools/workspace/spdlog:repository.bzl", "spdlog_repository")
spdlog_repository(name = "spdlog", mirrors = DEFAULT_MIRRORS)

load("//tools/workspace/common_robotics_utilities:repository.bzl", "common_robotics_utilities_repository")  # noqa

load("//tools/workspace/libfranka:repository.bzl", "libfranka_repository")
load("//tools/workspace/libfranka_common:repository.bzl", "libfranka_common_repository")  # noqa
load("//tools/workspace/libfranka_legacy:repository.bzl", "libfranka_legacy_repository")  # noqa
load("//tools/workspace/libfranka_common_legacy:repository.bzl", "libfranka_common_legacy_repository")  # noqa
load("//tools/workspace/poco:repository.bzl", "poco_repository")

common_robotics_utilities_repository(name = "common_robotics_utilities", mirrors = DEFAULT_MIRRORS)  # noqa
libfranka_repository(name = "libfranka", mirrors = DEFAULT_MIRRORS)
libfranka_common_repository(name = "libfranka_common", mirrors = DEFAULT_MIRRORS)  # noqa
libfranka_legacy_repository(name = "libfranka_legacy", mirrors = DEFAULT_MIRRORS)  # noqa
libfranka_common_legacy_repository(name = "libfranka_common_legacy", mirrors = DEFAULT_MIRRORS)  # noqa
poco_repository(name = "poco")
