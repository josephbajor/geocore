//! Facade-only lifecycle tests: no lower-layer crate is imported.

use kernel::{Error, Kernel, SessionPolicy};

#[test]
fn sessions_own_independent_parts_and_policy() {
    let configured = SessionPolicy::v1();
    let kernel = Kernel::with_default_policy(configured.clone());
    let mut first = kernel.create_session();
    let mut second = kernel.create_session();
    assert_eq!(first.policy(), &configured);
    assert_eq!(second.policy(), &configured);

    let first_part = first.create_part();
    let second_part = second.create_part();
    assert_eq!(format!("{first_part:?}"), "PartId(<opaque>)");
    assert_eq!(first.parts().len(), 1);
    assert_eq!(second.parts().len(), 1);
    assert!(matches!(first.part(second_part), Err(Error::UnknownPart)));

    let part = first.part(first_part.clone()).unwrap();
    assert_eq!(part.id(), first_part);
    assert_eq!(part.bodies().len(), 0);
    assert_eq!(part.regions().len(), 0);
    assert_eq!(part.shells().len(), 0);
    assert_eq!(part.faces().len(), 0);
    assert_eq!(part.loops().len(), 0);
    assert_eq!(part.fins().len(), 0);
    assert_eq!(part.edges().len(), 0);
    assert_eq!(part.vertices().len(), 0);
}

#[test]
fn removed_part_ids_are_stale_and_generation_safe() {
    let mut session = Kernel::new().create_session();
    let first = session.create_part();
    let stale = session.create_part();
    let third = session.create_part();
    session.remove_part(stale.clone()).unwrap();
    assert!(matches!(
        session.part(stale.clone()),
        Err(Error::UnknownPart)
    ));

    let replacement = session.create_part();
    assert_ne!(stale, replacement);
    assert!(matches!(session.edit_part(stale), Err(Error::UnknownPart)));
    assert_eq!(
        session.parts().collect::<Vec<_>>(),
        vec![first, replacement, third]
    );
}

#[test]
fn exclusive_part_capability_still_allows_read_views() {
    let mut session = Kernel::new().create_session();
    let id = session.create_part();
    let expected_policy = session.policy().clone();
    let edit = session.edit_part(id.clone()).unwrap();
    assert_eq!(edit.id(), id);
    assert_eq!(edit.policy(), &expected_policy);
    assert_eq!(edit.as_part().bodies().len(), 0);
}
