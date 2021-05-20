# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka",
        commit = "faa0219f286d857f01fa2097389ae6a23a4d9350",
        sha256 = "eefec7a65d2b2a80593ff85ef416fe090563e534012378b30c24c77d9e83db55",  # noqa
        build_file = "//tools/workspace/libfranka:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
