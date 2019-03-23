///////////////////////////////////////////
/// Boids class
///////////////////////////////////////////
///////////////////////////////////////////
/*
  Usage:

    b = Boids(numBoids: 10); // an initial number of boids. Flock size can be arbitrarily increased by using addBoid()
    f = {|thisInstance, boids|
      thisInstace.info; // print info about this flock
      boids.postln; // the list of BoidUnits
    };
    b.moveFlock(f);
*/


BoidsND {
  var <>dimensions, <>numBoids, <>timestep, <>centerInstinct, <>innerDistance, <>matchVelocity, <centerOfMass;
  var >boidList, <maxVelocity, workingMaxVelocity, <minSpace, targets, obstacles;
  var <bounds, <innerBoundRatio, <useInnerBounds, <innerBounds;

  *new {|dimensions = 3, numBoids = 10, timestep = 0.5, centerInstinct = 1, innerDistance = 1, matchVelocity = 1|
    ^super.newCopyArgs(dimensions, numBoids, timestep, centerInstinct, innerDistance, matchVelocity).init;
  }

  init {
    boidList = List.new(0); // an empty list of BoidUnits that we fill below
    maxVelocity = 5; // speed limit in meters per second (need to multiply it by the timestep)
    workingMaxVelocity = maxVelocity * timestep; // set the workingMaxVelocity (accounting for the timestep)
    minSpace = 1; // minmum distance between boids in a flock in meters
    centerOfMass = RealVector.zero(dimensions); // init center of mass at the origin
    // bounds = [[-500,500], [-500,500]]; // set the default bounds
    bounds = dimensions.collect{
      [-500, 500]; // 1000 meters (units)
    };
    useInnerBounds = false; // default to not using the inner bounds
    innerBoundRatio = 0.1; // default to 10%
    innerBounds = innerBoundRatio * bounds; // for ease of getting and setting
    this.prFillBoidList(numBoids); // fill the list with boids

    targets = List.new(0);
    obstacles = List.new(0);
  }

  // rule 1
  prGetCenterOfMass {
    var sum = RealVector.zero(dimensions); // a zero vector to add to
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
      vec = RealVector.zero(dimensions); // a new zero vector
      count = 1;
      boidList.do{|thisBoid|
        // var tmpVec = RealVector.zero(dimensions);
        // don't check for boids that are the exact same object
        if ((boid === thisBoid).not) {
          dist = boid.pos.dist(thisBoid.pos); // get the distance between these boids <--------------- fix this
          // if the absolute value of the distance is less than the threshold
          if (abs(dist) < minSpace) {
            ///// original ///////
            // vec = vec - ((boid.pos-thisBoid.pos)/abs(dist)); // calculate the difference vector
            /////////////////////

            vec = vec + ((boid.pos-thisBoid.pos)*(minSpace/(dist**2))); // calculate the difference vector
            // vec = vec - (thisBoid.pos-boid.pos); // calculate the difference vector
            // tmpVec = boid.pos-thisBoid.pos;
            // tmpVec = tmpVec.norm/abs(dist);
            // tmpVec = thisBoid.pos - boid.pos;
            // vec = vec + tmpVec;

            count = count+1;
          };
        };
      };
    vec = vec/count; // average
    boid.innerDistance = innerDistance * vec; // set the innerDistance vector in each BoidUnit
    };
  }

  // rule 3
  prGetVelocityMatch {
    var sum = RealVector.zero(dimensions); // a new zero vector
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
    // could instead pass an array of Nodes from which the NodeID's could be extracted and passed...
    num.do{
      var boid;
      boid = BoidUnitND.rand(dimensions, bounds, centerInstinct, innerDistance, matchVelocity, workingMaxVelocity)
        .bounds_(bounds); // make it
      boidList.add(boid); // add it to the list
    };
  }

  addBoid {|nodeID|
    var initPos, boid;
    // initPos = RealVector.rand(dimensions, bounds[0][0], bounds[0][1]); // get a random vector position
    // initPos = centerOfMass + initPos; // place it near the center of the flock
    initPos = centerOfMass; // place it near the center of the flock
    boid = BoidUnitND.new(dimensions, bounds, initPos, maxVelocity: workingMaxVelocity)
      .bounds_(bounds); // make it
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

    forkIfNeeded {
      func.(this); // evaluate the function while passing this instance
    };
  }

  // calculate the new values but don't send them to the BoidUnits
  calcFlock {|func|
    this.prGetCenterOfMass; // rule 1
    this.prGetInnerDistance; // rule 2
    this.prGetVelocityMatch; // rule 3

    func.(this); // evaluate the function while passing this instance
  }

  // set with a function instead of at once. Easier.
  setBoundLength {|dim, size|
    if(dim.isNil || size.isNil) {"Not enough args".error; ^nil}; // cry if something is wrong
    bounds[dim] = [size * -0.5, size * 0.5];
  }

  ///////////////////////////////////
  // targeting
  ///////////////////////////////////
  addTarget {|coordinates, gravity|
    if(coordinates.isNil || gravity.isNil)
      {"Insuffient arguments: %, %: no target was added!".format(coordinates, gravity).warn; ^this};
    targets.add([RealVector.newFrom(coordinates), gravity]);
  }

  clearTargets {
    targets.clear; // clear the list
  }

  removeTarget {|index|
    if(index.isNil) {
      targets.pop; // remove the last index
    } {
      targets.removeAt(index); // remove at the index
    };
  }

  editTarget {|index, coordinates, gravity|
    if(index.isNil) {"Index is nil: no targets were edited!".warn}; // throw a warning if insufficent args were supplied
    if(coordinates.notNil) {targets[index][0] = RealVector.newFrom(coordinates)}; // should check here if target is a Vector or not
    if(gravity.notNil) {targets[index][1] = gravity}; // edit the strength parameter
  }

  /////////////////////////////////////////
  ///// obstacles
  //////////////////////////////////////////
  addObstacle {|coordinates, gravity|
    if(coordinates.isNil or: gravity.isNil)
      {"Insuffient arguments: %, %: no obstacle was added!".format(coordinates, gravity).warn; ^this};
    obstacles.add([RealVector.newFrom(coordinates), gravity]);
  }

  clearObstacles {
    obstacles.clear; // clear the list
  }

  removeObstacle {|index|
    if(index.isNil) {
      obstacles.pop; // remove the last index
    } {
      obstacles.removeAt(index); // remove at the index
    };
  }

  editObstacle {|index, coordinates, gravity|
    if(index.isNil) {"Index is nil: no obstacles were edited!".warn}; // throw a warning if insufficent args were supplied
    if(coordinates.notNil) {obstacles[index][0] = RealVector.newFrom(coordinates)}; // should check here if target is a Vector or not
    if(gravity.notNil) {obstacles[index][1] = gravity}; // edit the strength parameter
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

  // innerBoundRatio_ {|val|
  //   if (val>0.95) {"Clipped innerBoundRatio to 0.95".warn}; // tell if we're doing something dumb
  //   innerBoundRatio = val.clip(0,0.95); // clip it
  //   innerBounds = bounds * innerBoundRatio; // for easer getting
  //   boidList.do(_.innerBounds_(innerBounds)); // set it in the BoidUnits internally
  // }

  // useInnerBounds_ {|boolean|
  //   useInnerBounds = boolean; // set it
  //   boidList.do(_.useInnerBounds_(useInnerBounds)); // set it for each BoidUnit
  // }

  // don't let us set the inner bounds manually (for now?)
  innerBounds_ {
    "Set innerBounds with innerBoundRatio!".warn;
    ^this;
  }

  // visualizer
  visualizer {|whichDimensions = #[0,1], showLabels = false, returnWindow = false|
    var window, loop, availableBounds, size, plotX, plotY;
    availableBounds = Window.availableBounds;
    size = availableBounds.width/3;
    window = Window("Dimensions: % : %".format(whichDimensions[0], whichDimensions[1]), Rect(availableBounds.width-size,availableBounds.height-size,size,size)).front;
    window.view.background_(Color.white);
    plotX = whichDimensions[0];
    plotY = whichDimensions[1];

    // draw the boids (as squares for now)
    window.drawFunc = {
      ////////
      // plot the boids as black squares ////////
      ////////
      boidList.do{|boid, i|
        var normalizedPos;
        Pen.color = Color.black;
        normalizedPos = [
          (boid.pos[plotX]+bounds[plotX][0].abs)/(bounds[plotX][0].abs*2),
          1- ((boid.pos[plotY]+bounds[plotY][0].abs)/(bounds[plotY][0].abs*2))
        ];
        // Pen.addOval(Rect(window.bounds.width*normalizedPos[0], window.bounds.height*normalizedPos[1], 5, 5));
        Pen.addWedge(
          Point(window.bounds.width*normalizedPos[0], window.bounds.height*normalizedPos[1]), // point
          10, // radius (pixels)
          (-1*atan2(boid.vel[plotY], boid.vel[plotX])) - 3.5342917352885, // start angle (angle - pi/8 - pi) for visualizer corrections
          0.78539816339745 // size of angle (pi/4)
        );
        if(showLabels) {
          Pen.stringAtPoint(i.asString, Point(window.bounds.width*normalizedPos[0] + 3, window.bounds.height*normalizedPos[1] + 3), color: Color.black);
        };
        Pen.perform(\fill);
      };

      ////////
      // plot the targets as blue squares
      ////////
      targets.do{|target, i|
        var normalizedPos, color;
        color = Color.fromHexString("4989FF");
        Pen.color = color;
        normalizedPos = [
          (target[0][plotX]+bounds[plotX][0].abs)/(bounds[plotX][0].abs*2),
          (target[0][plotY]+bounds[plotY][0].abs)/(bounds[plotY][0].abs*2)
        ];
        normalizedPos = [normalizedPos[0], 1 - normalizedPos[1]];

        Pen.addOval(
          Rect(window.bounds.width*normalizedPos[0], window.bounds.height*normalizedPos[1], 5, 5);
        );
        if(showLabels) {
          Pen.stringAtPoint(i.asString, Point(window.bounds.width*normalizedPos[0] + 3, window.bounds.height*normalizedPos[1] + 3), color: color);
        };
        Pen.perform(\fill);
      };

      ////////
      // plot the obstacles as red squares
      ////////
      obstacles.do{|obstacle, i|
        var normalizedPos, color;
        color = Color.fromHexString("FF4949");
        Pen.color = color;
        normalizedPos = [
          (obstacle[0][plotX]+bounds[plotX][0].abs)/(bounds[plotX][0].abs*2),
          (obstacle[0][plotY]+bounds[plotY][0].abs)/(bounds[plotY][0].abs*2)
        ];
        normalizedPos = [normalizedPos[0], 1 - normalizedPos[1]];

        Pen.addOval(
          Rect(window.bounds.width*normalizedPos[0], window.bounds.height*normalizedPos[1], 5, 5);
        );
        if(showLabels) {
          Pen.stringAtPoint(i.asString, Point(window.bounds.width*normalizedPos[0] + 3, window.bounds.height*normalizedPos[1] + 3), color: color);
        };
        Pen.perform(\fill);
      };

    };

    loop = {
      loop {window.refresh; timestep.wait};
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

  // alias for boidList
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
BoidUnitND {
  var <>dim, <bounds, <>pos, <>vel, <maxVelocity, <nodeID, <centerOfMass;
  var <>centerInstinct, <>innerDistance, <>matchVelocity;
  classvar halfCosTable;

  *initClass {
    halfCosTable = 2048.collect{|i|
    	i = i*(2048.reciprocal);
    	// (sin((i*pi)-(pi*0.5))*0.5)+0.5; // ascending
      (cos(i*pi)*0.5)+0.5; // descending
    };
  }

  *new {|dim, bounds, vel, pos, maxVelocity = 5|
    ^super.newCopyArgs(dim, bounds, pos, vel, maxVelocity).init(dim, bounds, nil, nil, nil, maxVelocity);
  }

  *rand {|dim, bounds, centerOfMass, innerDistance, matchVelocity, maxVelocity = 5|
    ^super.new.init(dim, bounds, centerOfMass, innerDistance, matchVelocity, maxVelocity);
  }

  init {|...args|
    dim = args[0]; // set dimensions
    bounds = args[1];
    maxVelocity = args[5] ? 10;
    vel = vel ? RealVector.rand(dim,-1*maxVelocity, maxVelocity);
    pos = pos ? RealVector.rand(dim,-1*bounds[0][0], bounds[0][0]); // this could be made better by taking the bounds of each dimension into account

    // if these are not set, set them
    centerOfMass = args[2] ? RealVector.rand(dim,-10,10);
    innerDistance = args[3] ? RealVector.rand(dim,-10,10);
    matchVelocity = args[4] ? RealVector.rand(dim,-10,10);

    centerInstinct = centerOfMass/100; // set this here
    vel = vel.limit(maxVelocity); // limit the size of the velocity vector
  }

  bound {
    var vec, boundVectors = List.new(0);

    dim.collect{|i|
      var amount = 0;
      if(pos[i] < bounds[i][0]) {
          amount = bounds[i][0] + pos[i].abs; // how far off are we
          amount = maxVelocity * (amount/maxVelocity).min(1);
        }
        {
          if(pos[i] > bounds[i][1]) {
            amount = bounds[i][1] - pos[i]; // how far off are we
            amount = maxVelocity * (amount/maxVelocity).min(1);
          };
        };
      boundVectors.add(amount); // add it to the list
    };

    vec = RealVector.newFrom(boundVectors.asArray); // get a new vector
    // pos = pos + vec; // add the vectors
    vel = vel + vec; // add the vectors
  }

  // get multidimensional distance
  getDist {|vec|
    var dist;
    if(vec.size != this.dim) {"Can't get distance; dimensions don't match.".error; ^nil};
    dist = dim.collect{|i|
      (this.pos[i] - vec[i])**2;
    };
    ^dist.sum.sqrt; // return the distance
  }

  moveBoid {|targets, obstacles|
    vel = vel + centerInstinct + innerDistance + matchVelocity; // sum the vectors and get a new velocity
    if (targets.isEmpty.not) {vel = vel + this.calcTargetsWithField(targets)}; // if there are targets, calculate the vector
    if (obstacles.isEmpty.not) {vel = vel + this.calcObstaclesWithField(obstacles)}; // if there are obstacles, calculate the vector
    this.bound; // bound the coordinates
    vel = vel.limit(maxVelocity); // speed limit
    pos = pos + vel; // get the new position
  }

  calcObstacles {|obstacles|
    var vec = RealVector.zero(dim);
    obstacles.do{|obstacle|
      vec = vec - ((obstacle[0]-pos)*obstacle[1]);
    };
    ^vec; // return the vector
  }

  calcObstaclesWithField {|obstacles|
    var vec = RealVector.zero(dim), distFromTarget, gravity, reweighted;
    obstacles.do{|obstacle|
      distFromTarget = pos.dist(obstacle[0]).max(1); // get the distance from this boid to the obstacle
      // if gravity is an array, apply gravities in specific dimensions
      if(obstacle[1].isArray) {
        reweighted = dim.collect{|i|
          gravity = this.inverseSquare(distFromTarget, obstacle[1][i]*10).clip(0,1);
          (obstacle[0][i]+pos[i])*gravity; // save it
        };
        vec = RealVector.newFrom(reweighted); // get the vector
        } {
          // else it's an integer so apply it evenly
          gravity = this.inverseSquare(distFromTarget, obstacle[1]).clip(0,1);
          vec = vec + ((obstacle[0]+pos)*gravity);
        };
      };
      ^vec; // return the vector
    }

  calcTargets {|targets|
    var vec = RealVector.zero(dim);
    targets.do{|target|
      vec = vec + ((target[0]-pos)*target[1]);
    };
    ^vec; // return the vector
  }

  calcTargetsWithField {|targets|
    var vec = RealVector.zero(dim), distFromTarget, gravity, reweighted;
    targets.do{|target|
      distFromTarget = pos.dist(target[0]).max(1); // get the distance from this boid to the target
      // if gravity is an array, apply gravities in specific dimensions
      if(target[1].isArray) {
        reweighted = dim.collect{|i|
          gravity = this.inverseSquare(distFromTarget, target[1][i]*1000).clip(0,1);
          (target[0][i]-pos[i])*gravity; // save it
        };
        vec = RealVector.newFrom(reweighted); // get the vector
      } {
        // else it's an integer so apply it evenly
        gravity = this.inverseSquare(distFromTarget, target[1]*100).clip(0,1);
        vec = vec + ((target[0]-pos)*gravity);
      };
    };
    ^vec; // return the vector
  }

  inverseSquare {|dist = 1, gravity = 1|
    ^(1*gravity)/(dist**2);
  }

  columbsLaw {|dist, q1, q2|
    var k = 8987551787.3681764;
    ^((q1*q2)/(dist**2))*k;
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
    // innerBounds = bounds * innerBoundRatio;
  }
}