# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_common_legacy_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka-common",
        commit = "277a0fc6ce3de03e13d8d537c245d9d95cdae215",
        sha256 = "b553bed95916f98922be8be3db0441745a7098f1822ae5bbf75ebce593f937f4",  # noqa
        build_file = "//tools/workspace/libfranka_common_legacy:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
