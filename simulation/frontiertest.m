jf = JFrontier(1, [ 0 1 2 3 ], [0 1 2 3], 1)
jf_2 = JFrontier(2, [ 1 2 3 4 ], [ 1 2 3 4 ], 2)
jf_3 = JFrontier(2, [ 1 2 3 5 ], [ 1 2 3 5 ], 2)
fn = FrontierNode(jf)
fn.insert(jf, jf_2)
fn.draw('out');

node = fn.findClosestNode(jf_2);
fprintf('%s\n',node.getContent().toString());

JFrontier.getDistance(0, 0, 1, 1)