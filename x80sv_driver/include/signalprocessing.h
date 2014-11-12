#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#define FILTER_ORDER 3

namespace SignalProcessing
{
/*
0.1 cutoff:

In [4]: butter(2, 0.1)
Out[4]: 
(array([ 0.02008337,  0.04016673,  0.02008337]),
 array([ 1.        , -1.56101808,  0.64135154]))

1/100th:

 In [5]: butter(2, 0.01)
 Out[5]: 
 (array([ 0.00024136,  0.00048272,  0.00024136]),
  array([ 1.        , -1.95557824,  0.95654368]))

*/
    class IirFilter
    {
        public:
        double _state[FILTER_ORDER];
        double _a[FILTER_ORDER];
        double _b[FILTER_ORDER];
        
        IirFilter()
        {
            /*
            _b[0] = 0.02008337;
            _b[1] = 0.04016673;
            _b[2] = 0.02008337;
            _a[0] = 1;
            _a[1] = -1.56101808;
            _a[2] = 0.64135154;
            */

            _b[0] = 0.00024136;
            _b[1] = 0.00048272;
            _b[2] = 0.00024136;
            _a[0] = 1;
            _a[1] = -1.95557824;
            _a[2] = 0.95654368;
        }

        double feed(double x)
        {
            // Shift last values of signal:
            for (int i=FILTER_ORDER; i > 0; i--)
            {
                _state[i] = _state[i - 1];
            }

            // Shift in current value:
            _state[0] = x;

            // Calculate new value:
            double fx = 0;
            for (int i=0; i<FILTER_ORDER; i++)
            {
                fx += _state[i] * _b[i];
            }

            for (int i=1; i<FILTER_ORDER; i++)
            {
                fx -= _state[i] * _a[i];
            }

            return fx;
        }
    };
}

#endif

