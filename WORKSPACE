# -*- python -*-

workspace(name = "drake_franka_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "deac84309f6d9fba05f2613b1a099daaefa2b7ca",
    "4d775fe0a9b016402398c390cbb99dd1416420639f8ca83ed795648bc6286895",
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

new_local_repository(
    name = "libpoco",
    path = "/usr",
    build_file = "tools/libpoco.BUILD.bazel",
)

load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("@drake//tools/workspace/rules_python:repository.bzl", "rules_python_repository")  # noqa
rules_python_repository(name = "rules_python", mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/gflags:repository.bzl", "gflags_repository")
gflags_repository(name = "gflags")

load("@drake//tools/workspace/glib:repository.bzl", "glib_repository")
glib_repository(name = "glib")

load("@drake//tools/workspace:github.bzl", "github_archive")

load("@drake//tools/workspace/lcm:repository.bzl", "lcm_repository")
lcm_repository(name = "lcm", mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/lcmtypes_bot2_core:repository.bzl", "lcmtypes_bot2_core_repository")  # noqa
lcmtypes_bot2_core_repository(name = "lcmtypes_bot2_core",
                              mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/lcmtypes_robotlocomotion:repository.bzl", "lcmtypes_robotlocomotion_repository")  # noqa
lcmtypes_robotlocomotion_repository(name = "lcmtypes_robotlocomotion",
                                    mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/python:repository.bzl", "python_repository")
python_repository(name = "python")

github_archive(
    name = "libfranka_common",
    repository = "frankaemika/libfranka-common",
    commit = "277a0fc6ce3de03e13d8d537c245d9d95cdae215",
    sha256 = "b553bed95916f98922be8be3db0441745a7098f1822ae5bbf75ebce593f937f4",
    build_file = "//tools:libfranka-common.BUILD.bazel",
    mirrors = DEFAULT_MIRRORS,
)

github_archive(
    name = "libfranka",
    repository = "frankaemika/libfranka",
    commit = "0.7.1",
    sha256 = "6a4ad0fa9451ddc6d2a66231ee8ede3686d6e3b67fd4cd9966ba30bdc82b9408",
    build_file = "//tools:libfranka.BUILD.bazel",
    mirrors = DEFAULT_MIRRORS,
)
