#! /bin/bash

tmp=$(mktemp -d)

echo "$tmp"
currdir=$(pwd)

cp -r src "$tmp"/.
cp -r LICENSE-MIT LICENSE-APACHE README.md "$tmp"/.

### Publish bevy_xpbd_2d
sed 's#\.\./\.\./src#src#g' crates/bevy_xpbd_2d/Cargo.toml > "$tmp"/Cargo.toml
cp -r crates/bevy_xpbd_2d/examples "$tmp"/.
cd "$tmp" && cargo publish

### Remove the 2D examples and return to previous directory
rm -rf examples
cd "$currdir" || exit

### Publish bevy_xpbd_3d
sed 's#\.\./\.\./src#src#g' crates/bevy_xpbd_3d/Cargo.toml > "$tmp"/Cargo.toml
cp -r crates/bevy_xpbd_3d/examples "$tmp"/.
cp -r crates/bevy_xpbd_3d/benches "$tmp"/.
cd "$tmp" && cargo publish

rm -rf "$tmp"