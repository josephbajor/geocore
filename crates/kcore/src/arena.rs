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
//! Copy-on-write undo frames provide rollback/partition savepoints without
//! cloning an entire session. A frame snapshots allocator metadata and saves
//! each pre-existing slot only on its first mutation; commit returns a
//! deterministic net mutation list, while rollback restores entity contents,
//! handle validity, free-list order, and subsequent allocation behavior.

use core::fmt;
use core::hash::{Hash, Hasher};
use core::marker::PhantomData;

use crate::error::{Error, Result};

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

#[derive(Clone)]
struct Slot<T> {
    generation: u32,
    value: Option<T>,
}

#[derive(Clone)]
struct UndoFrame<T> {
    slots_len: usize,
    free: Vec<u32>,
    live: usize,
    originals: Vec<(u32, Slot<T>)>,
}

/// Net kind of one arena mutation committed from an undo frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArenaChangeKind {
    /// A handle became live during the frame.
    Created,
    /// A handle that was live at frame entry was mutably accessed and
    /// remains live with the same identity.
    Modified,
    /// A handle that was live at frame entry is no longer live.
    Deleted,
}

/// One deterministic net mutation produced by committing an undo frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArenaChange<T> {
    handle: Handle<T>,
    kind: ArenaChangeKind,
}

impl<T> ArenaChange<T> {
    /// Affected handle. Deleted handles are intentionally stale after
    /// commit but retain the identity that existed at frame entry.
    pub fn handle(&self) -> Handle<T> {
        self.handle
    }

    /// Net mutation kind.
    pub fn kind(&self) -> ArenaChangeKind {
        self.kind
    }
}

/// A typed generational arena.
pub struct Arena<T> {
    slots: Vec<Slot<T>>,
    free: Vec<u32>,
    live: usize,
    undo: Vec<UndoFrame<T>>,
}

impl<T: Clone> Clone for Arena<T> {
    fn clone(&self) -> Self {
        Self {
            slots: self.slots.clone(),
            free: self.free.clone(),
            live: self.live,
            // A clone is an independent current-state snapshot, never a
            // second owner of the source arena's active rollback scopes.
            undo: Vec::new(),
        }
    }
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
            undo: Vec::new(),
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

impl<T: Clone> Arena<T> {
    fn record_slot(&mut self, index: u32) {
        if self.undo.is_empty() {
            return;
        }
        let original = self.slots[index as usize].clone();
        for frame in &mut self.undo {
            if index as usize >= frame.slots_len
                || frame
                    .originals
                    .iter()
                    .any(|(recorded, _)| *recorded == index)
            {
                continue;
            }
            frame.originals.push((index, original.clone()));
        }
    }

    /// Insert an entity, returning its handle.
    pub fn insert(&mut self, value: T) -> Handle<T> {
        self.live += 1;
        if let Some(index) = self.free.pop() {
            self.record_slot(index);
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

    /// Mutably borrow the entity behind a handle; `None` if the handle is stale.
    pub fn get_mut(&mut self, handle: Handle<T>) -> Option<&mut T> {
        if !self.contains(handle) {
            return None;
        }
        self.record_slot(handle.index);
        self.slots[handle.index as usize].value.as_mut()
    }

    /// Remove an entity, returning it; `None` if the handle is stale.
    /// The slot's generation is bumped so existing handles to it dangle.
    pub fn remove(&mut self, handle: Handle<T>) -> Option<T> {
        if !self.contains(handle) {
            return None;
        }
        self.record_slot(handle.index);
        let slot = &mut self.slots[handle.index as usize];
        let value = slot.value.take().expect("contains checked live slot");
        self.live -= 1;
        // On generation exhaustion the slot is retired (never freed) rather
        // than allowing a generation to repeat.
        if slot.generation < u32::MAX {
            slot.generation += 1;
            self.free.push(handle.index);
        }
        Some(value)
    }

    /// Begin a copy-on-write undo frame. Frames may be nested; every active
    /// frame observes subsequent mutations independently.
    pub fn begin_undo_frame(&mut self) {
        self.undo.push(UndoFrame {
            slots_len: self.slots.len(),
            free: self.free.clone(),
            live: self.live,
            originals: Vec::new(),
        });
    }

    fn undo_frame_changes(&self, frame: &UndoFrame<T>) -> Vec<ArenaChange<T>> {
        let mut originals: Vec<_> = frame.originals.iter().collect();
        originals.sort_by_key(|(index, _)| *index);
        let mut changes = Vec::new();
        for &(index, ref original) in originals {
            let current = &self.slots[index as usize];
            match (original.value.is_some(), current.value.is_some()) {
                (true, true) if original.generation == current.generation => {
                    changes.push(ArenaChange {
                        handle: Handle {
                            index,
                            generation: current.generation,
                            _marker: PhantomData,
                        },
                        kind: ArenaChangeKind::Modified,
                    });
                }
                (true, true) => {
                    changes.push(ArenaChange {
                        handle: Handle {
                            index,
                            generation: original.generation,
                            _marker: PhantomData,
                        },
                        kind: ArenaChangeKind::Deleted,
                    });
                    changes.push(ArenaChange {
                        handle: Handle {
                            index,
                            generation: current.generation,
                            _marker: PhantomData,
                        },
                        kind: ArenaChangeKind::Created,
                    });
                }
                (true, false) => changes.push(ArenaChange {
                    handle: Handle {
                        index,
                        generation: original.generation,
                        _marker: PhantomData,
                    },
                    kind: ArenaChangeKind::Deleted,
                }),
                (false, true) => changes.push(ArenaChange {
                    handle: Handle {
                        index,
                        generation: current.generation,
                        _marker: PhantomData,
                    },
                    kind: ArenaChangeKind::Created,
                }),
                (false, false) => {}
            }
        }
        for (index, slot) in self.slots.iter().enumerate().skip(frame.slots_len) {
            if slot.value.is_some() {
                changes.push(ArenaChange {
                    handle: Handle {
                        index: index as u32,
                        generation: slot.generation,
                        _marker: PhantomData,
                    },
                    kind: ArenaChangeKind::Created,
                });
            }
        }
        changes
    }

    /// Inspect the deterministic net mutations of the innermost undo frame
    /// without consuming it. A subsequent commit returns the identical list;
    /// rollback remains available after inspection.
    pub fn pending_undo_frame_changes(&self) -> Result<Vec<ArenaChange<T>>> {
        let frame = self.undo.last().ok_or(Error::TransactionInactive)?;
        Ok(self.undo_frame_changes(frame))
    }

    /// Commit the innermost undo frame and return its deterministic net
    /// mutations in slot order.
    pub fn commit_undo_frame(&mut self) -> Result<Vec<ArenaChange<T>>> {
        let frame = self.undo.pop().ok_or(Error::TransactionInactive)?;
        Ok(self.undo_frame_changes(&frame))
    }

    /// Roll back the innermost undo frame, restoring contents, identities,
    /// allocator state, and future handle allocation exactly.
    pub fn rollback_undo_frame(&mut self) -> Result<()> {
        let frame = self.undo.pop().ok_or(Error::TransactionInactive)?;
        self.slots.truncate(frame.slots_len);
        for (index, slot) in frame.originals {
            self.slots[index as usize] = slot;
        }
        self.free = frame.free;
        self.live = frame.live;
        Ok(())
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

    #[test]
    fn clone_preserves_handles_and_allocator_state() {
        let mut arena: Arena<i32> = Arena::new();
        let live = arena.insert(1);
        let removed = arena.insert(2);
        arena.remove(removed);

        let mut cloned = arena.clone();
        assert_eq!(cloned.get(live), Some(&1));
        let original_next = arena.insert(3);
        let cloned_next = cloned.insert(3);
        assert_eq!(original_next, cloned_next);

        *cloned.get_mut(live).unwrap() = 4;
        assert_eq!(arena.get(live), Some(&1));
        assert_eq!(cloned.get(live), Some(&4));
    }

    #[test]
    fn rollback_restores_contents_handle_validity_and_allocator_order() {
        let mut arena = Arena::new();
        let a = arena.insert(String::from("a"));
        let spare = arena.insert(String::from("spare"));
        arena.remove(spare).unwrap();
        let mut control = arena.clone();

        arena.begin_undo_frame();
        arena.get_mut(a).unwrap().push_str(" changed");
        let reused = arena.insert(String::from("reused"));
        let appended = arena.insert(String::from("appended"));
        arena.remove(a).unwrap();
        assert!(!arena.contains(a));
        assert!(arena.contains(reused));
        assert!(arena.contains(appended));
        arena.rollback_undo_frame().unwrap();

        assert_eq!(arena.get(a).map(String::as_str), Some("a"));
        assert!(!arena.contains(reused));
        assert!(!arena.contains(appended));
        let next = arena.insert(String::from("next"));
        let control_next = control.insert(String::from("next"));
        assert_eq!(next, control_next);
    }

    #[test]
    fn commit_reports_net_changes_in_slot_order() {
        let mut arena = Arena::new();
        let modified = arena.insert(10);
        let deleted = arena.insert(20);

        arena.begin_undo_frame();
        *arena.get_mut(modified).unwrap() = 11;
        arena.remove(deleted).unwrap();
        let replacement = arena.insert(30);
        let appended = arena.insert(40);
        let pending = arena.pending_undo_frame_changes().unwrap();
        let changes = arena.commit_undo_frame().unwrap();

        assert_eq!(pending, changes);

        assert_eq!(
            changes,
            vec![
                ArenaChange {
                    handle: modified,
                    kind: ArenaChangeKind::Modified,
                },
                ArenaChange {
                    handle: deleted,
                    kind: ArenaChangeKind::Deleted,
                },
                ArenaChange {
                    handle: replacement,
                    kind: ArenaChangeKind::Created,
                },
                ArenaChange {
                    handle: appended,
                    kind: ArenaChangeKind::Created,
                },
            ]
        );
    }

    #[test]
    fn mutation_preview_does_not_consume_rollback_state() {
        let mut arena = Arena::new();
        let value = arena.insert(1);
        arena.begin_undo_frame();
        *arena.get_mut(value).unwrap() = 2;
        assert_eq!(
            arena.pending_undo_frame_changes().unwrap(),
            vec![ArenaChange {
                handle: value,
                kind: ArenaChangeKind::Modified,
            }]
        );
        arena.rollback_undo_frame().unwrap();
        assert_eq!(arena.get(value), Some(&1));
        assert_eq!(
            arena.pending_undo_frame_changes().unwrap_err(),
            Error::TransactionInactive
        );
    }

    #[test]
    fn nested_frames_restore_their_own_entry_state() {
        let mut arena = Arena::new();
        let value = arena.insert(1);
        arena.begin_undo_frame();
        *arena.get_mut(value).unwrap() = 2;
        arena.begin_undo_frame();
        *arena.get_mut(value).unwrap() = 3;
        let inner = arena.commit_undo_frame().unwrap();
        assert_eq!(inner.len(), 1);
        assert_eq!(arena.get(value), Some(&3));
        arena.rollback_undo_frame().unwrap();
        assert_eq!(arena.get(value), Some(&1));

        arena.begin_undo_frame();
        *arena.get_mut(value).unwrap() = 4;
        arena.begin_undo_frame();
        *arena.get_mut(value).unwrap() = 5;
        arena.rollback_undo_frame().unwrap();
        assert_eq!(arena.get(value), Some(&4));
        arena.rollback_undo_frame().unwrap();
        assert_eq!(arena.get(value), Some(&1));
    }

    #[test]
    fn unbalanced_frames_are_errors_and_clones_are_independent_snapshots() {
        let mut arena = Arena::new();
        let value = arena.insert(1);
        assert_eq!(
            arena.commit_undo_frame().unwrap_err(),
            Error::TransactionInactive
        );
        assert_eq!(
            arena.rollback_undo_frame().unwrap_err(),
            Error::TransactionInactive
        );

        arena.begin_undo_frame();
        *arena.get_mut(value).unwrap() = 2;
        let mut snapshot = arena.clone();
        assert_eq!(snapshot.get(value), Some(&2));
        assert_eq!(
            snapshot.commit_undo_frame().unwrap_err(),
            Error::TransactionInactive
        );
        arena.rollback_undo_frame().unwrap();
        assert_eq!(arena.get(value), Some(&1));
        assert_eq!(snapshot.get(value), Some(&2));
    }
}
