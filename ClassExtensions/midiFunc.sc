+ MIDIFunc {

	ccNRPN {|ccNum, action|
		var list, seq, ccNum_in = 0, valIn = 0;
    	list = #[99, 98, 6, 38]; // the channels the 14-bit message (and it's ccNum) come in on
     	seq = Pseq(list, 1).asStream; // don't edit this yet... probably a more simple way to do this
     	CCResponder({arg src, chan, num, val;
        if(num == seq.next) {
          switch(num)
            {99} {ccNum_in = ccNum_in | (val << 7)}
            {98} {ccNum_in = ccNum_in | val}
            {6} {valIn = valIn | (val << 7) }
            {38} {
            	valIn = valIn | val;
              if(ccNum_in == ccNum) {
                action.value(ccNum_in, valIn);
              };
              ccNum_in = valIn = 0;
              seq.reset; // reset the sequence
            };
         } {
          seq.reset;
         };
     	}, num: list
		);
	}

	// ccNRPN {arg func, ccNum, chan, srcID, argTemplate, dispatcher;
	// 	var list, seq, ccNum_in = 0, valIn = 0, idx = 0;
  //   	list = #[99, 98, 6, 38]; // the channels the 14-bit message (and it's ccNum) come in on
  //    	seq = Pseq(list, 1).asStream; // don't edit this yet... probably a more simple way to do this
  //    	MIDIFunc.cc({|val, num, chan, src|
  //       if(num == list[idx]) {
  //         switch(num)
  //           {99} {ccNum_in = ccNum_in | (val << 7)}
  //           {98} {ccNum_in = ccNum_in | val}
  //           {6} {valIn = valIn | (val << 7) }
  //           {38} {
  //           	valIn = valIn | val;
  //             if(ccNum_in == ccNum) {
  //               func.value(ccNum_in, valIn);
  //             };
  //             ccNum_in = valIn = 0;
  //             // seq.reset; // reset the sequence
	// 						idx = 0;
  //           };
	//					 idx = idx+1;
  //        } {
  //         // seq.reset;
	// 				idx = 0;
  //        };
  //    	});
	// };

}
