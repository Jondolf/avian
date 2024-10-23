#! /bin/bash

currdir=$(pwd)

mkdir -p target/crates/avian2d/
mkdir -p target/crates/avian3d/

cp -r src target/crates/avian2d/.
cp -r src target/crates/avian3d/.
cp -r LICENSE-MIT LICENSE-APACHE README.md crates/avian2d/Cargo.toml target/crates/avian2d/.
cp -r LICENSE-MIT LICENSE-APACHE README.md crates/avian3d/Cargo.toml target/crates/avian3d/.
sed 's#\.\./\.\./src#src#g' target/crates/avian2d/Cargo.toml
sed 's#\.\./\.\./src#src#g' target/crates/avian3d/Cargo.toml > target/crates/avian3d/Cargo.toml
cp -r crates/avian2d/. target/crates/avian2d/.
cp -r crates/avian3d/. target/crates/avian3d/.
