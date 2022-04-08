#include <vector>
#include <map>
#include <iostream>
#include <fstream>

bool debug=false;
const int new_event_mark=0xFFFF;
const int comma_char=0xFBF7;

// class for managing QIE "frame"
// in which data for 8 channels and
// one time sample are packed.
//
class QIE{
 public:

  std::vector<int> adc{0,0,0,0,0,0,0,0}
  std::vector<int> tdc{0,0,0,0,0,0,0,0};
  int reserve=0, cid=0, cide=0, bc0=0;

  QIE(){}

  // method for retreive raw data for an
  // individual frame and unpacking it into the
  // data members of this class
  void add_data(std::vector<uint8_t> data){

    if ( data.size() != 12 ){
      std::cout << "OOPS! received " << data.size() << "/12 bytes" << std::endl;
      return;
    }
    
    bc0     =  (data[1]&1);
    cide    = ((data[1]>>1)&1);
    cid     = ((data[1]>>2)&3);
    reserve = ((data[1]>>4)&7);
    
    // stitch together TDC words
    uint16_t TDCs = (data[10]<<8)&data[11];
    
    // extract ADC and TDC values
    for ( int q = 0 ; q < 8 ; q++ ){
      adc.push_back(data[q+2]);
      tdc.push_back(( TDCs>>(q*2) )&3);
    }
  
  }
  
  // helper function for debugging
  void print(){
    std::cout << "[[QIE]]" << std::endl;
    std::cout << "reserve: "  << reserve << std::endl;
    std::cout << "Cap ID: " << cid << std::endl;
    std::cout << "error: " << cide << std::endl;
    std::cout << "BC0: " << bc0 << std::endl;
    for ( int i = 0 ; i < adc.size() ; i ++ ) {
      std::cout << " ADC " << i << " : " << adc[i] ;
      
    }
    std::cout << std::endl;
    for ( int i = 0 ; i < tdc.size() ; i ++ ) {
      std::cout << " TDC " << i << " : " << tdc[i];
    }
    std::cout << std::endl;
  }
  
};


// class for organizing event
// data and managing the low-level
// unpacking of data
class TSevent{

 public:

  // and event is two vectors for QIE frames
  // qie1_ : frames for fiber 1 and 0<=elecID_<=7
  // qie2_ : frames for fiber 1 and 8<=elecID_<=15
  //
  // each element represents a different time samples of an event
  // there are typically 32 times samples, but data corruptions
  // can cause less
  std::vector<QIE> qie1_,qie2_;
  uint64_t time=0; 
  TSevent(std::vector<uint16_t> fiber1,
          std::vector<uint16_t> fiber2){

    // extract time since the start of spill from
    // the event
    time|=uint64_t(fiber2[1]);
    time|=uint64_t(fiber1[1])<<16;
    time|=uint64_t(fiber2[0])<<32;
    time|=uint64_t(fiber1[0])<<48;
    if ( debug ) std::cout << "time: " << std::hex << time << std::endl;
    
    // strip frames until the first bc
    // skipping first two words
    //
    // The data format is most easily decoded with 8 bit words.
    // So, the 16 bit words are split before being passed to
    // the QIE unpacker.
    std::vector<uint8_t> buffer;
    std::vector<uint8_t> res_buffer;
    // a flag used to keep track of partial frames that can
    // occur at the beginning of an event.
    bool first = true;
    int iword = 0;

    for (auto word : fiber1 ){
      if( debug ){
        std::cout << "buffer size: " << buffer.size() << std::endl;
        std::cout << "word: " << std::hex << word << std::endl;
        std::cout << "first byte: " << std::hex << (word&0xFF) << std::endl;
      }
      
      //skip first two words -- already handled above with the time extraction
      if( iword < 2 ){
        if( debug ) std::cout << "skip first two words" << std::endl;
        iword++;
        continue;
      }
      
      //skip words between 2 and first 0xBC (or 0xFC which occurs every 3564 time samples)
      if( first && !start_of_time_sample(word) ){
        if( debug ) std::cout << "skip words until first 0xBC" << std::endl;
        res_buffer.push_back(word&0xF);
        res_buffer.push_back((word>>8)&0xF);
        iword++;
        continue;
      }

      //for each 0xBC (or 0xFC) form new QIE object and clear buffer
      if( !first && start_of_time_sample(word) ){
        if( debug ) std::cout << "new time sample" << std::endl;
        QIE temp_qie;
        temp_qie.add_data(buffer);
        //temp_qie.print();
        qie1_.push_back(temp_qie);
        buffer.clear();
      }

      //mark occurence of first 0xBC (or 0xFC)
      if( first && start_of_time_sample(word) ){
        if( debug ) std::cout << "first fournd" << std::endl;
        first =false;
      }
      buffer.push_back(word&0xF);
      buffer.push_back((word>>8)&0xF);
      iword++;
    }

    // it looks like the extra words at the begging and
    // end of the event exactly form one extra frame,
    // which could be due to a pointer misalignment in a
    // circular buffer in the f/w???
    if( buffer.size() + res_buffer.size() == 12 ){
      for( auto word : res_buffer ){
        buffer.push_back(word);
      }
      QIE temp_qie;
      temp_qie.add_data(buffer);
      qie1_.push_back(temp_qie);
    }
    buffer.clear();
    res_buffer.clear();
    
    // repeat for data on fiber 2
    for (auto word : fiber2 ){
      if( debug ){
        std::cout << "buffer size: " << buffer.size() << std::endl;
        std::cout << "word: " << std::hex << word << std::endl;
        std::cout << "first byte: " << std::hex << (word&0xFF) << std::endl;
      }

      //skip first two words -- already handled above with the time extraction
      if( iword < 2 ){
        if( debug ) std::cout << "skip first two words" << std::endl;
        iword++;
        continue;
      }

      //skip words between 2 and first 0xBC (or 0xFC which occurs every 3564 time samples)
      if( first && !start_of_time_sample(word)){
        if( debug ) std::cout << "skip words until first 0xBC" << std::endl;
        res_buffer.push_back(word&0xF);
        res_buffer.push_back((word>>8)&0xF);
        iword++;
        continue;
      }
      //for each 0xBC (or 0xFC) form new QIE object and clear buffer
      if( !first && start_of_time_sample(word)){
        if( debug ) std::cout << "new time sample" << std::endl;
        QIE temp_qie;
        temp_qie.add_data(buffer);
        //temp_qie.print();
        qie2_.push_back(temp_qie);
        buffer.clear();
      }
      //mark occurence of first 0xBC (or 0xFC)
      if( first && start_of_time_sample(word)){
        if( debug ) std::cout << "first fournd" << std::endl;
        first =false;
      }
      buffer.push_back(word&0xF);
      buffer.push_back((word>>8)&0xF);
      iword++;
    }

    if( buffer.size() + res_buffer.size() == 12 ){
      for( auto word : res_buffer ){
        buffer.push_back(word);
      }
      QIE temp_qie;
      temp_qie.add_data(buffer);
      qie2_.push_back(temp_qie);
    }
    buffer.clear();
    res_buffer.clear();

    // in case there is a mismatch, remove
    // frames until fiber1 and fiber2 data
    // matches.  There should be 32 frames for
    // each, though.
    while( qie1_.size() > qie2_.size() ){
      qie1_.erase(qie1_.begin());
    }
    while( qie2_.size() > qie1_.size() ){
      qie2_.erase(qie2_.begin());
    }
    
  }

  // function for standardizing the comma characters
  // the represent the start of a frame
  bool start_of_time_sample(uint16_t word){
    return ( (word&0xFF) == 0xBC ) || ( (word&0xFF) == 0xFC );
  }

  // helper function for debugging
  void print(){
    std::cout << "[[TSevent]]" << std::endl;
    std::cout << "Time: " << std::hex << time << std::endl;
    std::cout << "FIBER1 (" << qie1_.size() << " time samples)" << std::endl;
    for( auto q : qie1_ ){
      q.print();
    }
    std::cout << "FIBER2 (" << qie2_.size() << " time samples)" << std::endl;
    for( auto q : qie2_ ){
      q.print();
    }
  }
  
};
  
// code for stripping comma characters from data
// and locating event delimiters
int main(int argc, const char *argv[])
{

  // open binary file which is passed via command line
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << "<filename>\n";
    return 1;
  }

  // run_buffer* stores all data for individual fibers for an
  // entire run
  std::vector<std::vector<uint16_t>> run_buffer1, run_buffer2;
  // event_buffer* stores all daata for one event for one fiber
  std::vector<uint16_t> event_buffer1,event_buffer2;

  std::ifstream in(argv[1], std::ios::binary);
  // flag used to keep track of extraneous words in a UDP packet
  // before the start of the event
  bool preamble=true;

  // loop over data
  while (in) {

    // buffers for individual words for each data fiber
    // 16 bit buffers are convenient for removing pad
    // words from the data stream
    uint16_t fiber1a,fiber2a,fiber1b,fiber2b;
    
    in.read((char*)&fiber1a,sizeof(fiber1a)); // extract 16 bits
    in.read((char*)&fiber1b,sizeof(fiber1b)); // extract 16 bits
    in.read((char*)&fiber2a,sizeof(fiber2a)); // extract 16 bits
    in.read((char*)&fiber2b,sizeof(fiber2b)); // extract 16 bits

    // check whether a new_event_mark is found.  If yes, push event buffers
    // to run buffers and clear event buffers
    if ( fiber1a == new_event_mark &&
         fiber1b == new_event_mark &&
         fiber2a == new_event_mark &&
         fiber2b == new_event_mark ){
      if ( preamble ){
        preamble=false; // start buffering event
      }else{            // end of event, push event buffer into run buffer
        run_buffer1.push_back(event_buffer1);
        run_buffer2.push_back(event_buffer2);
        event_buffer1.clear();
        event_buffer2.clear();
      }
      continue;
    }
    
    // check if we're in an event -- initially we won't be but after the
    // first time the new_event_mark is seen in each fiber, this will
    // start filling the event buffers
    if ( !preamble ){
      
      // remove all comma chraacters before filling buffer 
      if( fiber1a != comma_char){
        event_buffer1.push_back(fiber1a);
      }
      if( fiber1b != comma_char ){
        event_buffer1.push_back(fiber1b);
      }
      if( fiber2a != comma_char ){
        event_buffer2.push_back(fiber2a);
      }
      if( fiber2b != comma_char ){
        event_buffer2.push_back(fiber2b);
      }
    }
  }

  // done organizing raw data...

  // now run buffers can be looped over and events can be digitized
  // into TSevent objects


  std::cout << "Found " << run_buffer1.size() << ", "
            << run_buffer2.size() << " events"
            << std::endl;


  // loop over events
  for( int i = 0 ; i < run_buffer1.size() ; i++ ){
    std::cout << "Event " << i << " has " << run_buffer1[i].size() << ", " << run_buffer2[i].size() << " words" << std::endl;

    // unpack event
    TSevent evt(run_buffer1[i],run_buffer2[i]);
    evt.print();
    
   }

   return 0;
}
