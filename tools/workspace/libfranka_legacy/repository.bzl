# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_legacy_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka",
        commit = "0.7.1",
        sha256 = "6a4ad0fa9451ddc6d2a66231ee8ede3686d6e3b67fd4cd9966ba30bdc82b9408",  # noqa
        build_file = "//tools/workspace/libfranka_legacy:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
