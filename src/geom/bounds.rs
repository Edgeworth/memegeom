use crate::primitive::Rt;
use crate::primitive::point::Pt;

#[must_use]
pub fn pt_cloud_bounds(pts: &[Pt]) -> Option<Rt> {
    let first = pts.first()?;
    let mut bl = *first;
    let mut tr = *first;
    for pt in pts {
        bl.x = bl.x.min(pt.x);
        bl.y = bl.y.min(pt.y);
        tr.x = tr.x.max(pt.x);
        tr.y = tr.y.max(pt.y);
    }
    Some(Rt::enclosing(bl, tr))
}

#[must_use]
pub fn rt_cloud_bounds<I: IntoIterator<Item = Rt>>(rts: I) -> Option<Rt> {
    rts.into_iter().reduce(|a, b| a.united(&b))
}
