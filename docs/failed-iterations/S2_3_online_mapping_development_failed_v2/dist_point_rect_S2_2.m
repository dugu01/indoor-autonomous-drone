function d = dist_point_rect_S2_2(p, rect)
% DIST_POINT_RECT_S2_2  Signed distance from 2-D point to axis-aligned rectangle.
x = p(1); y = p(2);
ox = rect(1); oy = rect(2); ow = rect(3); oh = rect(4);
dx = max([ox-x, 0, x-(ox+ow)]);
dy = max([oy-y, 0, y-(oy+oh)]);
outside = hypot(dx,dy);
inside = x >= ox && x <= ox+ow && y >= oy && y <= oy+oh;
if inside
    d = -min([x-ox, ox+ow-x, y-oy, oy+oh-y]);
else
    d = outside;
end
end
