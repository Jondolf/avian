//! Miscallaneous utility functions.

pub(crate) use bevy::platform::time::Instant;

/// A helper function for iterating over a slice in parallel or serially
/// based on the `parallel` feature.
///
/// If `slice.len() < min_len`, serial iteration will be used.
///
/// The `ComputeTaskPool` is used if parallelism is enabled.
///
/// # Example
///
/// ```ignore
/// let mut slice = vec![1, 2, 3, 4];
///
/// par_for_each(&mut slice, |index, item| {
///     *item += index;
/// });
///
/// assert_eq!(slice, vec![1, 3, 5, 7]);
/// ```
#[inline(always)]
pub fn par_for_each<T, F>(mut slice: &mut [T], min_len: usize, f: F)
where
    T: Send + Sync,
    F: Fn(usize, &mut T) + Send + Sync,
{
    #[cfg(not(feature = "parallel"))]
    slice.iter_mut().enumerate().for_each(f);

    #[cfg(feature = "parallel")]
    {
        let task_pool_ = bevy::tasks::ComputeTaskPool::get();

        if task_pool_.thread_num() == 1 || slice.len() < min_len {
            slice.iter_mut().enumerate().for_each(|(index, item)| {
                f(index, item);
            });
        } else {
            // TODO: Is there a better approach than `par_chunk_map_mut`?
            let chunk_size_ = (slice.len() / task_pool_.thread_num()).max(1);
            bevy::tasks::ParallelSliceMut::par_chunk_map_mut(
                &mut slice,
                task_pool_,
                chunk_size_,
                |chunk_index_, chunk_| {
                    let index_offset_ = chunk_index_ * chunk_size_;
                    chunk_.iter_mut().enumerate().for_each(|(i, item)| {
                        let index = index_offset_ + i;
                        f(index, item);
                    });
                },
            );
        }
    }
}
