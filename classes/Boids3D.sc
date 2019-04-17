///////////////////////////////////////////
/// Boids class
///////////////////////////////////////////
///////////////////////////////////////////
/*
  Usage:

    b = Boids(numBoids: 10); // an initial number of boids. Flock size can be arbitrarily increased by using addBoid()
    f = {|thisInstance|
      thisInstace.info; // print info about this flock
      boids.postln; // the list of BoidUnits
    };
    b.moveFlock(f);
*/


Boids3D {
  var <numBoids, <>timestep, <>centerInstinct, <>innerDistance, <>matchVelocity, <centerOfMass;
  var >boidList, <maxVelocity, workingMaxVelocity, <minSpace, targets, obstacles;
  var <bounds;

  *new {|numBoids = 10, timestep = 0.5, centerInstinct = 1, innerDistance = 1, matchVelocity = 1|
    ^super.newCopyArgs(numBoids, timestep, centerInstinct, innerDistance, matchVelocity).init;
  }

  init {
    boidList = List.new(0); // an empty list of BoidUnits that we fill below
    maxVelocity = 5; // speed limit in meters per second (need to multiply it by the timestep)
    workingMaxVelocity = maxVelocity * timestep; // set the workingMaxVelocity (accounting for the timestep)
    minSpace = 1; // minmum distance between boids in a flock in meters
    centerOfMass = RealVector3D.zero; // init center of mass at the origin
    bounds = [[-500,500], [-500,500], [-500,500]]; // set the default bounds
    this.prFillBoidList(numBoids); // fill the list with boids
    targets = List.new(0);
    obstacles = List.new(0);
  }

  // rule 1
  prGetCenterOfMass {
    var sum = RealVector3D.zero; // a zero vector to add to
    boidList.do{|boid, i|
      sum = sum + boid.pos; // sum the values
    };
    centerOfMass = sum/boidList.size; // get the average and set it

    // now set the average within each BoidUnit and compensate for its percieved center by subtracting itself
    boidList.do{|boid, i|
      boid.centerOfMass = centerInstinct * ((sum - boid.pos)/(boidList.size - 1)); // set it
    };
  }

  // rule 2
  prGetInnerDistance {
    boidList.do{|boid|
      var vec, dist, count;
      vec = RealVector3D.newFrom([0,0]); // a new zero vector
      count = 1; // count the number of boids nearby to scale
      boidList.do{|thisBoid|
        // don't check for boids that are the exact same object
        if ((boid === thisBoid).not) {
          dist = boid.pos.dist(thisBoid.pos); // get the distance between these boids
          // if the absolute value of the distance is less than the threshold
          if (abs(dist) < minSpace) {
            ///// original ///////
            // vec = vec - ((boid.pos-thisBoid.pos)/abs(dist)); // calculate the difference vector
            vec = vec + ((boid.pos-thisBoid.pos)*(minSpace/(dist**2))); // calculate the difference vector
            /////////////////////
            count = count+1; // keep counting the boids in the vicinity
          };
        };
      };
      vec = vec/count; // average
      boid.innerDistance = innerDistance * vec; // set the innerDistance vector in each BoidUnit
    };
  }

  // rule 3
  prGetVelocityMatch {
    var sum = RealVector3D.zero; // a new zero vector
    // sum the velocities
    boidList.do{|boid|
      sum = sum + boid.vel;
    };
    boidList.do{|boid|
      var thisSum = sum - boid.vel; // remove this boid from the sum
      boid.matchVelocity = matchVelocity * ((thisSum/(boidList.size-1)) * 0.125); // send one eigth of the magnitude to the boid
    };
  }

  prFillBoidList {|num|
    num.do{
      var boid;
      boid = BoidUnit3D.rand(bounds, centerInstinct, innerDistance, matchVelocity, workingMaxVelocity);
      boidList.add(boid); // add it to the list
    };
  }

  addBoid {|initPos|
    var boid, initVel;
    initPos = initPos ? centerOfMass; // place it near the center of the flock
    initVel = RealVector3D.newFrom(Array.fill(3, {rrand(0.0,3.0)})); // random velocity
    boid = BoidUnit3D.new(initVel, initPos, bounds, centerOfMass, workingMaxVelocity);
    boidList.add(boid); // add it
  }

  removeBoid {|index|
    if (index.isNil) {
      boidList.pop; // if no arg, remove the last BoidUnit
    } {
      boidList.removeAt(index); // else, remove at the index
    };
  }

  sizeOfFlock {
    ^boidList.size; // return the size of the flock
  }

  moveFlock {|func|
    this.prGetCenterOfMass; // rule 1
    this.prGetInnerDistance; // rule 2
    this.prGetVelocityMatch; // rule 3

    // method to set tell all the boids to now calculate and move?
    boidList.do{|boid|
      boid.moveBoid(targets, obstacles); // tell the boid to calculate and move it's position
    };
    func.(this); // evaluate the function while passing this instance
  }

  // calculate the new values but don't send them to the BoidUnits
  calcFlock {|func|
    this.prGetCenterOfMass; // rule 1
    this.prGetInnerDistance; // rule 2
    this.prGetVelocityMatch; // rule 3
    func.(this); // evaluate the function while passing this instance
  }

  getPanVals {
    ^boidList.collect{|boid|
      boid.getPanVals; // get the pan values
    };
  }

  // creates a rectangle of size [dim]
  bounds_ {|dim|
    var rect, xLen, yLen, zLen;
    if(dim.isArray) {
      if(dim.size==3) {
        // we're good
        xLen = dim[0];
        yLen = dim[1];
        zLen = dim[2];
      };
    } {
      "dim must be a two element array".error; // else something is wrong
      ^nil; // return nill
    };

    // create the bounds of a rectangle with the given dimensions with the origin at the center
    rect = [[-0.5*xLen, 0.5*xLen], [-0.5*yLen, 0.5*yLen], [-0.5*zLen, 0.5*zLen]];
    bounds = rect; // set the new bounds
    // set the bounds in each BoidUnit
    boidList.do{|boid|
      boid.bounds = bounds; // set it in each Boid
    };
  }

  ///////////////////////////////////
  // targeting
  ///////////////////////////////////
  addTarget {|pos, gravity|
    if(pos.isNil or: gravity.isNil)
      {"Insuffient arguments: %, %: no target was added!".format(pos, gravity).warn; ^this};
    targets.add(Dictionary.with(*[\pos->RealVector3D.newFrom(pos[..1]), \strength->gravity]));
  }

  clearTargets {
    targets = targets.clear; // clear the list
  }

  removeTarget {|index|
    if(index.isNil) {
      targets.pop; // remove the last index
    } {
      targets.removeAt(index); // remove at the index
    };
  }

  editTarget {|index, pos, gravity|
    if(index.isNil) {"Index is nil: no targets were edited!".warn}; // throw a warning if insufficent args were supplied
    if(pos.notNil) {targets[index].add(\pos->RealVector3D.newFrom(pos[..1]))}; // should check here if target is a Vector or not
    if(gravity.notNil) {targets[index].add(\strength->gravity)}; // edit the gravity parameter
  }

  /////////////////////////////////////////
  ///// obstacles
  //////////////////////////////////////////
  addObstacle {|pos, repulsion|
    if(pos.isNil or: repulsion.isNil)
      {"Insuffient arguments: %, %: no obstacle was added!".format(pos, repulsion).warn; ^this};
    obstacles.add(Dictionary.with(*[\pos->RealVector3D.newFrom(pos[..1]), \strength->repulsion]));
  }

  clearObstacles {
    obstacles = List[]; // clear the list
  }

  removeObstacle {|index|
    if(index.isNil) {
      obstacles.pop; // remove the last index
    } {
      obstacles.removeAt(index); // remove at the index
    };
  }

  editObstacle {|index, obstacle, repulsion|
    if(index.isNil) {"Index is nil: no obstacles were edited!".warn}; // throw a warning if insufficent args were supplied
    if(obstacle.notNil) {obstacles[index].add(\pos->RealVector3D.newFrom(obstacle[..1]))}; // should check here if target is a Vector or not
    if(repulsion.notNil) {obstacles[index].add(\strength->repulsion)}; // edit the repulsion parameter
  }

  // print the variable information for this flock
  info {
    var str = "Boid Info::::\n";
    str = str ++ "\tnumBoids: %\n".format(numBoids);
    str = str ++ "\ttimestep: % s\n".format(timestep);
    str = str ++ "\tcenterInstinct: %\n".format(centerInstinct);
    str = str ++ "\tinnerDistance: %\n".format(innerDistance);
    str = str ++ "\tmatchVelocity: %\n".format(matchVelocity);
    str = str ++ "\tmaxVelocity: % m/s\n".format(maxVelocity);
    str = str ++ "\tminSpace: % m\n".format(minSpace);
    str.postln; // print it
  }

  /////////////////////////////////
  // custom setter methods
  /////////////////////////////////
  maxVelocity_ {|val|
    maxVelocity = val; // maxVelocity is the maximum length of the velocity vector
    workingMaxVelocity = maxVelocity * timestep;
    boidList.do{|boid|
      boid.maxVelocity = workingMaxVelocity; // set it in each individual boid
    };
  }

  minSpace_ {|val|
    minSpace = val;
    this.prGetInnerDistance;
  }

  // visualizer
  visualizer {|whichDimensions = #[0,1], showLabels = false, returnWindow = false, refreshInterval|
    var window, loop, availableBounds, getNormalizedPos, makeCircle, makeLabel, size, plotX, plotY;
    availableBounds = Window.availableBounds;
    size = availableBounds.width/3;
    window = Window("Dimensions: % : %".format(whichDimensions[0], whichDimensions[1]), Rect(availableBounds.width-size,availableBounds.height-size,size,size)).front;
    window.view.background_(Color.white);

    // get the dimensions
    plotX = whichDimensions[0];
    plotY = whichDimensions[1];

    // functions
    getNormalizedPos = {|pos|
      [(pos[0]+bounds[0][0].abs)/(bounds[0][0].abs*2), 1 - ((pos[1]+bounds[1][0].abs)/(bounds[1][0].abs*2))];
    };

    makeCircle = {|normalizedPos|
      Pen.addOval(Rect(window.bounds.width*normalizedPos[0], window.bounds.height*normalizedPos[1], 5, 5));
    };

    makeLabel = {|label, normalizedPos, color|
      Pen.stringAtPoint(label.asString, Point(window.bounds.width*normalizedPos[0] + 3, window.bounds.height*normalizedPos[1] + 3), color: color);
    };

    // draw the boids
    window.drawFunc = {
      // plot the boids as black squares
      boidList.do{|boid, i|
        var normalizedPos, color;
        color = Color.black;
        Pen.color = color;
        normalizedPos = getNormalizedPos.(boid.pos[plotX..plotY]);
        Pen.addWedge(
          Point(window.bounds.width*normalizedPos[0], window.bounds.height*normalizedPos[1]), // point
          10, // radius (pixels)
          (-1*atan2(boid.vel[plotY], boid.vel[plotX])) - 3.5342917352885, // start angle (angle - pi/8 - pi) for visualizer corrections
          0.78539816339745 // size of angle (pi/4)
        );
        if(showLabels) {
          makeLabel.(i, normalizedPos, color);
        };
        Pen.perform(\fill);
      };

      // plot the targets as blue squares
      targets.do{|target, i|
        var normalizedPos, color;
        color = Color.fromHexString("4989FF");
        Pen.color = color;
        normalizedPos = getNormalizedPos.(target.at(\pos)[plotX..plotY]);
        makeCircle.(normalizedPos); // make the circle
        if(showLabels) {makeLabel.(i, normalizedPos, color)};
        Pen.perform(\fill);
      };

      ////////
      // plot the obstacles as red squares
      ////////
      obstacles.do{|obstacle, i|
        var normalizedPos, color;
        color = Color.fromHexString("FF4949");
        Pen.color = color;
        normalizedPos = getNormalizedPos.(obstacle.at(\pos)[plotX..plotY]);
        makeCircle.(normalizedPos);
        if(showLabels) {makeLabel.(i, normalizedPos, color)};
        Pen.perform(\fill);
      };

    };

    loop = {
      refreshInterval = refreshInterval ? timestep;
      loop {window.refresh; refreshInterval.wait};
    }.fork(AppClock);

    window.onClose_({loop.stop});
    if(returnWindow) {^window};
  }

  /////////////////////////////
  // custom getter methods
  /////////////////////////////
  boidList {
    ^boidList.asArray;
  }

  boids {
    ^boidList.asArray;
  }

  targets {
    ^targets.asArray;
  }

  obstacles {
    ^obstacles.asArray;
  }

}

////////////////////////////////////////////////////////////
// not directly used but rather used by Boids
////////////////////////////////////////////////////////////
BoidUnit3D {
  var <>vel, <>pos, <bounds, <centerOfMass, <maxVelocity;
  var <>centerInstinct, <>innerDistance, <>matchVelocity;

  *new {|vel, pos, bounds, centerOfMass, maxVelocity = 5|
    ^super.newCopyArgs(vel, pos, bounds, centerOfMass, maxVelocity).init;
  }

  *rand {|bounds, centerOfMass, innerDistance, matchVelocity, maxVelocity = 5|
    ^super.new.init(bounds, centerOfMass, innerDistance, matchVelocity, maxVelocity);
  }

  init {|...args|
    bounds = bounds ? args[0] ? [[-500,500],[-500,500]]; // [ [xmin, xmax], [ymin, ymax]]
    vel = vel ? RealVector3D.newFrom(Array.fill(3, {rrand(0.0,3.0)}));
    pos = pos ? RealVector.rand(3, bounds[0][0],bounds[0][1]).asRealVector3D;
    maxVelocity = maxVelocity ? args[4] ? 5; // max velocity

    // if these are not set, set them
    centerOfMass = args[1] ? RealVector.rand(3, -10,10).asRealVector3D;
    innerDistance = args[2] ? RealVector.rand(3, -10,10).asRealVector3D;
    matchVelocity = args[3] ? RealVector.rand(3, -10,10).asRealVector3D;

    centerInstinct = centerOfMass/100; // set this here
    vel = vel.limit(maxVelocity); // limit the size of the velocity vector
  }

  bound {
    var vec = RealVector3D.zero; // a zero vector
    3.do{|i|
      var amount;
      if(pos[i] < bounds[i][0]) {
        amount = bounds[i][0] + pos[i].abs; // how far off are we
        vec[i] = amount; // change zero for this
      } {
        if(pos[i] > bounds[i][1]) {
          amount = bounds[i][1] - pos[i]; // how far off are we
          vec[i] = amount; // change zero for this
        };
      };
    };
    vel = vel + vec; // add the vectors in velocity-space
  }

  moveBoid {|targets, obstacles|
    if (targets.isEmpty.not) {vel = vel + this.calcTargets(targets)}; // if there are targets, calculate the vector
    if (obstacles.isEmpty.not) {vel = vel + this.calcObstacles(obstacles)}; // if there are obstacles, calculate the vector
    vel = vel + centerInstinct + innerDistance + matchVelocity; // sum the vectors and get a new velocity
    this.bound; // bound the coordinates
    vel = vel.limit(maxVelocity); // speed limit
    pos = pos + vel; // get the new position
  }

  getPanVals {
    var zero = RealVector3D.zero;
    ^[pos.theta, pos.dist(zero)]; // return the angle in radians and the distance from the origin
  }

  calcObstacles {|obstacles|
    var vec = RealVector3D.zero;
    obstacles.do{|obstacle|
      vec = this.prCalcVec(obstacle, vec, \obstacle);
    };
    ^vec; // return the vector
  }

  calcTargets {|targets|
    var vec = RealVector3D.zero;
    targets.do{|target|
      vec = this.prCalcVec(target, vec, \target);
    };
    ^vec; // return the vector
  }

  prCalcVec {|object, vec, type|
    var distFromTarget, diff, gravity;
    distFromTarget = pos.dist(object.at(\pos)).max(0.001); // get the distance from the object
    switch (type)
      {\target} {
        diff = object.at(\pos)-pos; // get the diff
        gravity = ((object.at(\strength)*100)/distFromTarget).max(0); // 1/r
      }
      {\obstacle} {
        diff = pos-object.at(\pos); // get the diff
        gravity = this.prInverseSquare(distFromTarget, object.at(\strength)*1000).max(0); // 1/r^2
      };
    ^vec + ((diff/diff.norm)*gravity); // return
  }

  /////////////////////////////////
  // gravity/repulsion scaling functions
  /////////////////////////////////
  prInverseSquare {|dist = 1, gravity = 1|
    ^gravity/(dist**2);
  }

  prArcTan {|dist = 1, gravity = 1, scalar = 10|
    gravity = gravity.reciprocal*scalar;
    dist = (dist*gravity)-gravity;
    gravity = atan(-1*dist);
    ^(gravity/3)+0.5;
  }

  prArcTan2 {|dist = 1, gravity = 1, scalar = 5|
    // scalar is where the arctangent function passes through 0 normally
    dist = (dist-(gravity*scalar));
    gravity = atan(-1*dist*gravity.reciprocal);
    ^(gravity/3)+0.5;
  }

  /////////////////////////////////
  // custom setter methods
  /////////////////////////////////
  centerOfMass_ {|vec|
    centerOfMass = vec; // get the perceived center of mass for this BoidUnit
    // each time we get a new center of mass, recalculate the first vector offset
    centerInstinct = centerOfMass/100; // get the vector that moves it 1% toward the center of the flock (this can be weighted??)
  }

  maxVelocity_ {|val|
    maxVelocity = val; // set it
    vel = vel.limit(maxVelocity); // limit it if it's bigger
  }

  bounds_ {|val|
    bounds = val;
  }
}