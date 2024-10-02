/// Creates a `const fn num_variants() -> usize` function for an enum.
macro_rules! num_variants {
    ($( #[$attrs:meta] )* $vis:vis enum $name:ident {
        $( $( #[$variant_attrs:meta] )* $variant:ident $(= $value:expr)? ),* $(,)?
    }) => {
        $(#[$attrs])* $vis enum $name {
            $(
                $(#[$variant_attrs])* $variant $(= $value)?
            ),*
        }
        impl $name {
            #[allow(unused, non_snake_case)]
            $vis const fn num_variants() -> usize {
                let mut count = 0;
                {
                    $( let $variant = (); count += 1; )*
                }
                count
            }
        }
    }
}
