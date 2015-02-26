#ifndef UserDataType_h
#define UserDataType_h
#define ADC_DIM 11
struct data_t {
  unsigned long time;
  int adc[ADC_DIM];
};
#endif  // UserDataType_h
