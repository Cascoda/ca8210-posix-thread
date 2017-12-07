#!/bin/bash
dir=$(cd -P -- "$(dirname -- "$0")" && pwd -P)

git submodule update --init --recursive

