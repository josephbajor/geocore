//! Generational arena storage for kernel entities.
//!
//! All kernel entities (curves, surfaces, faces, edges, …) live in typed
//! arenas and are referenced by copyable [`Handle`]s — never by pointers or
//! references. This gives the kernel:
//!
//! - **Stale-handle safety**: removing an entity bumps its slot's generation,
//!   so old handles observably dangle ([`Arena::get`] returns `None`) instead
//!   of aliasing a recycled slot.
//! - **Cheap identity**: handles are 8 bytes, hashable, and type-tagged, and
//!   map directly onto the integer entity tags of a PK-style C API.
//! - **Deterministic iteration**: iteration order is slot order, a pure
//!   function of the insertion/removal history — never hash order.
//!
//! Rollback/partition snapshots and journaling (spec §L2/L6) will be built on
//! this storage model.

use core::fmt;
use core::hash::{Hash, Hasher};
use core::marker::PhantomData;

/// A typed, generational reference to an entity in an [`Arena<T>`].
pub struct Handle<T> {
    index: u32,
    generation: u32,
    _marker: PhantomData<fn() -> T>,
}

// Manual impls: derives would wrongly bound on `T`.
impl<T> Clone for Handle<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Copy for Handle<T> {}
impl<T> PartialEq for Handle<T> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index && self.generation == other.generation
    }
}
impl<T> Eq for Handle<T> {}
impl<T> Hash for Handle<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.index.hash(state);
        self.generation.hash(state);
    }
}
impl<T> fmt::Debug for Handle<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Handle({}v{})", self.index, self.generation)
    }
}

struct Slot<T> {
    generation: u32,
    value: Option<T>,
}

/// A typed generational arena.
pub struct Arena<T> {
    slots: Vec<Slot<T>>,
    free: Vec<u32>,
    live: usize,
}

impl<T> Default for Arena<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Arena<T> {
    /// Empty arena.
    pub fn new() -> Self {
        Arena {
            slots: Vec::new(),
            free: Vec::new(),
            live: 0,
        }
    }

    /// Number of live entities.
    pub fn len(&self) -> usize {
        self.live
    }

    /// True if no entities are live.
    pub fn is_empty(&self) -> bool {
        self.live == 0
    }

    /// Insert an entity, returning its handle.
    pub fn insert(&mut self, value: T) -> Handle<T> {
        self.live += 1;
        if let Some(index) = self.free.pop() {
            let slot = &mut self.slots[index as usize];
            debug_assert!(slot.value.is_none());
            slot.value = Some(value);
            return Handle {
                index,
                generation: slot.generation,
                _marker: PhantomData,
            };
        }
        let index = u32::try_from(self.slots.len()).expect("arena exceeded u32 capacity");
        self.slots.push(Slot {
            generation: 0,
            value: Some(value),
        });
        Handle {
            index,
            generation: 0,
            _marker: PhantomData,
        }
    }

    fn slot(&self, handle: Handle<T>) -> Option<&Slot<T>> {
        self.slots
            .get(handle.index as usize)
            .filter(|s| s.generation == handle.generation && s.value.is_some())
    }

    /// True if the handle refers to a live entity.
    pub fn contains(&self, handle: Handle<T>) -> bool {
        self.slot(handle).is_some()
    }

    /// Borrow the entity behind a handle; `None` if the handle is stale.
    pub fn get(&self, handle: Handle<T>) -> Option<&T> {
        self.slot(handle).and_then(|s| s.value.as_ref())
    }

    /// Mutably borrow the entity behind a handle; `None` if the handle is stale.
    pub fn get_mut(&mut self, handle: Handle<T>) -> Option<&mut T> {
        self.slots
            .get_mut(handle.index as usize)
            .filter(|s| s.generation == handle.generation)
            .and_then(|s| s.value.as_mut())
    }

    /// Remove an entity, returning it; `None` if the handle is stale.
    /// The slot's generation is bumped so existing handles to it dangle.
    pub fn remove(&mut self, handle: Handle<T>) -> Option<T> {
        let slot = self
            .slots
            .get_mut(handle.index as usize)
            .filter(|s| s.generation == handle.generation)?;
        let value = slot.value.take()?;
        self.live -= 1;
        // On generation exhaustion the slot is retired (never freed) rather
        // than allowing a generation to repeat.
        if slot.generation < u32::MAX {
            slot.generation += 1;
            self.free.push(handle.index);
        }
        Some(value)
    }

    /// Iterate live entities in slot order (deterministic).
    pub fn iter(&self) -> impl Iterator<Item = (Handle<T>, &T)> {
        self.slots.iter().enumerate().filter_map(|(i, s)| {
            s.value.as_ref().map(|v| {
                (
                    Handle {
                        index: i as u32,
                        generation: s.generation,
                        _marker: PhantomData,
                    },
                    v,
                )
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_get_remove_roundtrip() {
        let mut arena: Arena<&str> = Arena::new();
        let a = arena.insert("a");
        let b = arena.insert("b");
        assert_eq!(arena.len(), 2);
        assert_eq!(arena.get(a), Some(&"a"));
        assert_eq!(arena.remove(a), Some("a"));
        assert_eq!(arena.len(), 1);
        assert_eq!(arena.get(b), Some(&"b"));
    }

    #[test]
    fn stale_handle_dangles_after_slot_reuse() {
        let mut arena: Arena<i32> = Arena::new();
        let a = arena.insert(1);
        arena.remove(a);
        let b = arena.insert(2); // reuses the slot with a new generation
        assert_eq!(a.index, b.index);
        assert!(!arena.contains(a));
        assert_eq!(arena.get(a), None);
        assert_eq!(arena.remove(a), None);
        assert_eq!(arena.get(b), Some(&2));
    }

    #[test]
    fn handles_are_value_types() {
        let mut arena: Arena<i32> = Arena::new();
        let a = arena.insert(7);
        let a2 = a;
        assert_eq!(a, a2);
        *arena.get_mut(a).unwrap() = 8;
        assert_eq!(arena.get(a2), Some(&8));
    }

    #[test]
    fn iteration_is_slot_ordered_and_skips_dead() {
        let mut arena: Arena<i32> = Arena::new();
        let h: Vec<_> = (0..5).map(|i| arena.insert(i)).collect();
        arena.remove(h[1]);
        arena.remove(h[3]);
        let values: Vec<i32> = arena.iter().map(|(_, &v)| v).collect();
        assert_eq!(values, vec![0, 2, 4]);
    }
}
