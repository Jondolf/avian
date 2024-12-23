//! A container for data associated with entities in a generational arena.

// Adapted from Rapier's `Coarena`, using `Entity` instead of `Index`.
// https://github.com/dimforge/rapier/blob/cf77b5bf574f8363794f979510deec5c08e58401/src/data/coarena.rs

use bevy::prelude::Entity;

/// A container for data associated with entities in a generational arena.
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Default)]
pub struct EntityDataIndex<T> {
    data: Vec<(u32, T)>,
}

impl<T> EntityDataIndex<T> {
    /// Creates a new [`EntityIndex`] with estimated capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
        }
    }

    /// Iterates through all elements of the entity index.
    pub fn iter(&self) -> impl Iterator<Item = (Entity, &T)> {
        self.data
            .iter()
            .enumerate()
            .filter(|(_, elt)| elt.0 != u32::MAX)
            .map(|(index, elt)| (entity_from_index_and_gen(index as u32, elt.0), &elt.1))
    }

    /// Gets a specific element from the entity index without specifying its generation number.
    ///
    /// It is strongly encouraged to use [`EntityIndex::get`] instead of this method because this method
    /// can suffer from the ABA problem.
    pub fn get_unknown_gen(&self, index: u32) -> Option<&T> {
        self.data.get(index as usize).map(|(_, t)| t)
    }

    /// Deletes an element from the entity index and returns its value.
    ///
    /// The stored value will be reset to the given `removed_value`.
    pub fn remove(&mut self, entity: Entity, removed_value: T) -> Option<T> {
        let (index, gen) = (entity.index(), entity.generation());
        let data = self.data.get_mut(index as usize)?;
        if gen == data.0 {
            data.0 = u32::MAX; // invalidate the generation number.
            Some(std::mem::replace(&mut data.1, removed_value))
        } else {
            None
        }
    }

    /// Gets a specific element from the entity index, if it exists.
    pub fn get(&self, entity: Entity) -> Option<&T> {
        let (index, gen) = (entity.index(), entity.generation());
        self.data
            .get(index as usize)
            .and_then(|(gg, t)| if gen == *gg { Some(t) } else { None })
    }

    /// Gets a mutable reference to a specific element from the entity index, if it exists.
    pub fn get_mut(&mut self, entity: Entity) -> Option<&mut T> {
        let (i, g) = (entity.index(), entity.generation());
        self.data
            .get_mut(i as usize)
            .and_then(|(gg, t)| if g == *gg { Some(t) } else { None })
    }

    /// Inserts an element into this entity index.
    pub fn insert(&mut self, entity: Entity, value: T)
    where
        T: Clone + Default,
    {
        let (index, gen) = (entity.index(), entity.generation());

        if self.data.len() <= index as usize {
            self.data
                .resize(index as usize + 1, (u32::MAX, T::default()));
        }

        self.data[index as usize] = (gen, value);
    }

    /// Ensures that the given element exists in this entity index, and return a mutable reference to it.
    pub fn ensure_element_exists(&mut self, entity: Entity, default: T) -> &mut T
    where
        T: Clone,
    {
        let (index, gen) = (entity.index(), entity.generation());

        if self.data.len() <= index as usize {
            self.data
                .resize(index as usize + 1, (u32::MAX, default.clone()));
        }

        let data = &mut self.data[index as usize];

        if data.0 != gen {
            *data = (gen, default);
        }

        &mut data.1
    }

    /// Ensure that elements at the two given indices exist in this entity index, and return their references.
    ///
    /// Missing elements are created automatically and initialized with the `default` value.
    pub fn ensure_pair_exists(
        &mut self,
        entity1: Entity,
        entity2: Entity,
        default: T,
    ) -> (&mut T, &mut T)
    where
        T: Clone,
    {
        let (index1, gen1) = (entity1.index(), entity1.generation());
        let (index2, gen2) = (entity2.index(), entity2.generation());

        assert_ne!(index1, index2, "Cannot index the same object twice.");

        let (elt1, elt2) = if index1 > index2 {
            if self.data.len() <= index1 as usize {
                self.data
                    .resize(index1 as usize + 1, (u32::MAX, default.clone()));
            }

            let (left, right) = self.data.split_at_mut(index1 as usize);
            (&mut right[0], &mut left[index2 as usize])
        } else {
            // i2 > i1
            if self.data.len() <= index2 as usize {
                self.data
                    .resize(index2 as usize + 1, (u32::MAX, default.clone()));
            }

            let (left, right) = self.data.split_at_mut(index2 as usize);
            (&mut left[index1 as usize], &mut right[0])
        };

        if elt1.0 != gen1 {
            *elt1 = (gen1, default.clone());
        }

        if elt2.0 != gen2 {
            *elt2 = (gen2, default);
        }

        (&mut elt1.1, &mut elt2.1)
    }
}

/// Creates an entity from an index and a generation number.
pub fn entity_from_index_and_gen(index: u32, generation: u32) -> Entity {
    Entity::from_bits((generation as u64) << 32 | index as u64)
}
