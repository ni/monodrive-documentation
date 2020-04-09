#!/bin/bash

# constants
ARGS_SHORT=s
ARGS_LONG=serve

# Build the Python Client documentation using pydoc
function build_python_docs()
{
  CLIENT_SRC="./client_source/simulator-python-client/"
  BUILD_DIR="./client_source/simulator-python-client/_build/pydocmd/monodrive/"
  CLIENT_DST="./docs/python_client/"

  echo -e "====== Building python documentation using pydocmd build..."
  pip install -e ${CLIENT_SRC}
  pip install -r ${CLIENT_SRC}/requirements.txt
  pushd ${CLIENT_SRC}
  pydocmd build
  popd

  echo -e "====== Moving Python Client docs to documentation directory..."
  mkdir -p ${CLIENT_DST}
  cp -R ${BUILD_DIR}* ${CLIENT_DST}
  echo -e "====== Python Client documentation built."
}

# Build the root documentation
function build_docs()
{
  mkdocs build
}

# Serve up the root documentation
function serve_docs()
{
  mkdocs serve
}
# setup exit on fail
set -e
set -o pipefail

# command line args
options=$(getopt --options ${ARGS_SHORT} --long ${ARGS_LONG} --name ${0} -- ${@})
eval set -- ${options}
while true; do
    case ${1} in
        -s | --serve )
            serve=true
            shift;;
        -- )
            shift
            break;;
        *) echo "unknown flag ${*}"; exit;;
    esac
done
serve=${serve:-false}

# run main
build_python_docs

if [ ${serve} == true ] ; then
  echo "====== Serving documentation on localhost"
  serve_docs
else
  echo "====== Building mkdocs documentation"
  build_docs
fi
