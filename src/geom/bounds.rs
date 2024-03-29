use crate::primitive::point::Pt;
use crate::primitive::rect::Rt;

pub fn pt_cloud_bounds(pts: &[Pt]) -> Rt {
    if pts.is_empty() {
        Rt::default()
    } else {
        let mut bl = pts[0];
        let mut tr = pts[0];
        for pt in pts {
            bl.x = bl.x.min(pt.x);
            bl.y = bl.y.min(pt.y);
            tr.x = tr.x.max(pt.x);
            tr.y = tr.y.max(pt.y);
        }
        Rt::enclosing(bl, tr)
    }
}

pub fn rt_cloud_bounds<I: IntoIterator<Item = Rt>>(rts: I) -> Rt {
    let mut b = Rt::empty();
    for r in rts {
        b = b.united(&r);
    }
    b
}
