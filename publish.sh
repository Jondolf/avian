#! /bin/bash

tmp=$(mktemp -d)

echo "$tmp"
currdir=$(pwd)

cp -r src "$tmp"/.
cp -r LICENSE-MIT LICENSE-APACHE README.md "$tmp"/.

### Publish avian2d
sed 's#\.\./\.\./src#src#g' crates/avian2d/Cargo.toml > "$tmp"/Cargo.toml
cp -r crates/avian2d/examples "$tmp"/.
cd "$tmp" && cargo publish

### Remove the 2D examples and return to previous directory
rm -rf examples
cd "$currdir" || exit

### Publish avian3d
sed 's#\.\./\.\./src#src#g' crates/avian3d/Cargo.toml > "$tmp"/Cargo.toml
cp -r crates/avian3d/examples "$tmp"/.
cp -r crates/avian3d/benches "$tmp"/.
cd "$tmp" && cargo publish

rm -rf "$tmp"