#!/bin/bash
# ------------------------------------------------------------------
# [Lukas Samel] Cyc script to embed opencyc 4.0 to the opencyc knowrob_addons repo.
#          Check the opencyc package for the opencyc ontology and download it in 
#		   case it cannot be localized.
# ------------------------------------------------------------------

VERSION=0.1
SUBJECT=some-unique-id
USAGE="Usage: command -ihv args"

# --- Options processing -------------------------------------------
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $CURRENT_DIR
cd ../

if [ ! -d opencyc-4.0 ]; then
  wget https://downloads.sourceforge.net/project/opencyc-backups/opencyc-4.0/opencyc-4.0-linux.tgz
  tar -xvzf opencyc-4.0-linux.tgz
  rm opencyc-4.0-linux.tgz
fi

cd opencyc-4.0/scripts/
./run-cyc.sh

