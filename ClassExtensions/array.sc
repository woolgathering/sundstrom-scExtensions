+ Array {

  // interpolate between two values, adding noise to the interpolated segments if desired.
  *noiseInterpolate {|start = 0, end = 1, size = 5, noise = 0.1|
    var array, tmp;
    array = [start,end]; // make an array
    array = array.resamp1(size); // get a new array that's linearly interpoalted
    tmp = array[1..array.size-2].collectInPlace{|val|
      val*noise.rand2 + val; // add some noise to the segments
    };
    tmp.do{|val, i| array[i+1] = val}; // return the values to the array
    ^array; // return the array
  }

  // write to file. Array.writeFile gets weird so I wrote this one.
  writeToFile {|path, delimiter = "\n", func|
    var file;
    file = File.open(path, "w"); // open a file
    this.do{|val, i|
      if(func.notNil) {
        file.write(func.(val, i) ++ delimiter); // write it
      } {
        file.write(val.asString ++ delimiter); // write it
      }
    };
    file.close;
  }

}
