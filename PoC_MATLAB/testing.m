pt = [0 0 0];
dir = [1 0 0 1];
h = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3));
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])

xfm = makehgtform('xrotate',pi/3,'yrotate',pi/5,'zrotate',pi/2);
newdir = xfm * dir';
h.UData = newdir(1);
h.VData = newdir(2);
h.WData = newdir(3);

for theta = linspace(0,pi,64)
  for phi = linspace(-pi,pi,64)
    for psi = linspace(0,2*pi,64)
      xfm = makehgtform('xrotate',theta,'yrotate',phi,'zrotate',psi);
      newdir = xfm * dir';
      h.UData = newdir(1);
      h.VData = newdir(2);
      h.WData = newdir(3);
      drawnow
    end
  end
end