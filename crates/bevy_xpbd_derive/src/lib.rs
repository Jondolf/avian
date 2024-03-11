//! Provides derive implementations for Bevy XPBD.

use proc_macro::TokenStream;

use quote::{quote, quote_spanned};
use syn::{parse_macro_input, spanned::Spanned, Data, DeriveInput};

// Modified macro From the discontinued Heron,
// see https://github.com/jcornaz/heron/blob/main/macros/src/lib.rs
#[proc_macro_derive(PhysicsLayer)]
pub fn derive_physics_layer(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let enum_ident = input.ident;

    fn non_enum_item_error(span: proc_macro2::Span) -> TokenStream {
        quote_spanned! { span =>  compile_error!("only enums can automatically derive PhysicsLayer"); }
                .into()
    }
    let variants = match &input.data {
        Data::Enum(data) => &data.variants,
        Data::Struct(data) => {
            return non_enum_item_error(data.struct_token.span);
        }
        Data::Union(data) => {
            return non_enum_item_error(data.union_token.span);
        }
    };

    if variants.len() > 32 {
        return quote! { compile_error!("PhysicsLayer only supports a maximum of 32 layers"); }
            .into();
    }

    let to_bits_result: Result<Vec<_>, _> = variants
        .iter()
        .enumerate()
        .map(|(index, variant)| {
            if !variant.fields.is_empty() {
                return Err(variant.fields.span());
            }
            let bits: u32 = 1 << index;
            let ident = &variant.ident;

            Ok(quote! { #enum_ident::#ident => #bits, })
        })
        .collect();

    let to_bits = match to_bits_result {
        Ok(tokens) => tokens,
        Err(span) => return quote_spanned! { span => compile_error!("can only derive PhysicsLayer for enums without fields"); }.into(),
    };

    let all_bits: u32 = if variants.len() == 32 {
        0xffffffff
    } else {
        (1 << variants.len()) - 1
    };

    let expanded = quote! {
        #[cfg(feature = "2d")]
        use bevy_xpbd_2d::prelude::PhysicsLayer;
        #[cfg(feature = "3d")]
        use bevy_xpbd_3d::prelude::PhysicsLayer;

        impl PhysicsLayer for #enum_ident {
            fn all_bits() -> u32 {
                #all_bits
            }

            fn to_bits(&self) -> u32 {
                match self {
                    #(#to_bits)*
                }
            }
        }
    };

    TokenStream::from(expanded)
}
