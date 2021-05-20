# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_common_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka-common",
        commit = "405524b9469712bffc3442873b6e7828593a5b4b",
        sha256 = "60fc0f59d6e14fe3e9963c194b952bcf2f883798415424100711a9e93d5089c7",  # noqa
        build_file = "//tools/workspace/libfranka_common:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
