//! Named deterministic façade identity iterators.

use core::iter::FusedIterator;

use crate::{
    BodyId, CurveId, EdgeId, FaceId, FinId, LoopId, PartId, PcurveId, RegionId, ShellId, SurfaceId,
    VertexId,
};

macro_rules! id_iterator {
    ($name:ident, $id:ty, $description:literal) => {
        #[doc = $description]
        pub struct $name<'a> {
            inner: Box<dyn Iterator<Item = $id> + 'a>,
            remaining: usize,
        }

        impl<'a> $name<'a> {
            pub(crate) fn new(iter: impl Iterator<Item = $id> + 'a, remaining: usize) -> Self {
                Self {
                    inner: Box::new(iter),
                    remaining,
                }
            }
        }

        impl Iterator for $name<'_> {
            type Item = $id;

            fn next(&mut self) -> Option<Self::Item> {
                if self.remaining == 0 {
                    return None;
                }
                let next = self.inner.next();
                if next.is_some() {
                    self.remaining -= 1;
                } else {
                    self.remaining = 0;
                }
                next
            }

            fn size_hint(&self) -> (usize, Option<usize>) {
                (self.remaining, Some(self.remaining))
            }
        }

        impl ExactSizeIterator for $name<'_> {
            fn len(&self) -> usize {
                self.remaining
            }
        }

        impl FusedIterator for $name<'_> {}

        impl core::fmt::Debug for $name<'_> {
            fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                formatter
                    .debug_struct(stringify!($name))
                    .field("remaining", &self.remaining)
                    .finish_non_exhaustive()
            }
        }
    };
}

id_iterator!(
    BodyIds,
    BodyId,
    "Deterministically ordered body identities."
);
id_iterator!(
    PartIds,
    PartId,
    "Deterministically ordered part identities."
);
id_iterator!(
    RegionIds,
    RegionId,
    "Deterministically ordered region identities."
);
id_iterator!(
    ShellIds,
    ShellId,
    "Deterministically ordered shell identities."
);
id_iterator!(
    FaceIds,
    FaceId,
    "Deterministically ordered face identities."
);
id_iterator!(
    LoopIds,
    LoopId,
    "Deterministically ordered loop identities."
);
id_iterator!(FinIds, FinId, "Deterministically ordered fin identities.");
id_iterator!(
    EdgeIds,
    EdgeId,
    "Deterministically ordered edge identities."
);
id_iterator!(
    VertexIds,
    VertexId,
    "Deterministically ordered vertex identities."
);
id_iterator!(
    CurveIds,
    CurveId,
    "Deterministically ordered 3D curve geometry identities."
);
id_iterator!(
    SurfaceIds,
    SurfaceId,
    "Deterministically ordered surface geometry identities."
);
id_iterator!(
    PcurveIds,
    PcurveId,
    "Deterministically ordered parameter-space curve geometry identities."
);
