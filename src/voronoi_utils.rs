use super::TBError;
use boostvoronoi::diagram as VD;
use std::collections::VecDeque;

/// Mark infinite edges and their adjacent edges as EXTERNAL.
pub(crate) fn reject_external_edges(
    diagram: &VD::Diagram<i64, f64>,
) -> Result<yabf::Yabf, TBError> {
    let mut rejected_edges = yabf::Yabf::with_capacity(diagram.edges().len());

    for edge in diagram.edges().iter() {
        let edge = edge.get();
        let edge_id = edge.id();

        if diagram.edge_is_infinite(edge_id)? {
            mark_connected_edges(diagram, edge_id, &mut rejected_edges)?;
        }
    }
    Ok(rejected_edges)
}

/// Marks this edge and all other edges connecting to it via vertex1.
/// Repeat stops when connecting to input geometry.
/// if 'initial' is set to true it will search both ways, edge and the twin edge.
/// 'initial' will be set to false when going past the first edge
/// Note that this is not a recursive function (as it is in boostvoronoi)
pub(crate) fn mark_connected_edges(
    diagram: &VD::Diagram<i64, f64>,
    edge_id: VD::EdgeIndex,
    marked_edges: &mut yabf::Yabf,
) -> Result<(), TBError> {
    let mut initial = true;
    let mut queue = VecDeque::<VD::EdgeIndex>::new();
    queue.push_front(edge_id);

    'outer: while !queue.is_empty() {
        // unwrap is safe since we just checked !queue.is_empty()
        let edge_id = queue.pop_back().unwrap();

        if marked_edges.bit(edge_id.0) {
            initial = false;
            continue 'outer;
        }

        let v1 = diagram.edge_get_vertex1(edge_id)?;
        if diagram.edge_get_vertex0(edge_id)?.is_some() && v1.is_none() {
            // this edge leads to nowhere
            marked_edges.set_bit(edge_id.0, true);
            initial = false;
            continue 'outer;
        }
        marked_edges.set_bit(edge_id.0, true);

        #[allow(unused_assignments)]
        if initial {
            initial = false;
            queue.push_back(diagram.edge_get_twin(edge_id)?);
        } else {
            marked_edges.set_bit(diagram.edge_get_twin(edge_id)?.0, true);
        }

        if v1.is_none()
            || !diagram.edges()[(Some(edge_id))
                .ok_or_else(|| TBError::InternalError("Could not get edge twin".to_string()))?
                .0]
                .get()
                .is_primary()
        {
            // stop traversing this line if vertex1 is not found or if the edge is not primary
            initial = false;
            continue 'outer;
        }
        // v1 is always Some from this point on
        if let Some(v1) = v1 {
            let v1 = diagram.vertex_get(v1)?.get();
            if v1.is_site_point() {
                // stop iterating line when site points detected
                initial = false;
                continue 'outer;
            }
            //self.reject_vertex(v1, color);
            let mut edge_iter = v1.get_incident_edge()?;
            let v_incident_edge = edge_iter;
            loop {
                if !marked_edges.bit(edge_iter.0) {
                    queue.push_back(edge_iter);
                }
                edge_iter = diagram.edge_rot_next(edge_iter)?;
                if edge_iter == v_incident_edge {
                    break;
                }
            }
        }
        initial = false;
    }
    Ok(())
}
