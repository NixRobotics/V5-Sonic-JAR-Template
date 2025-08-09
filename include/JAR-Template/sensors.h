using namespace vex;

class myinertial: public inertial {
    public:
    
        myinertial(int32_t index) : inertial(index) {
        }
    
        double temperature()
        {
            return inertial::temperature();
        }
        
};

class myrotation: public rotation {
public:

    float gear_ratio = 1;
    
    myrotation(int32_t index, bool reverse = false) : rotation(index, reverse) {

    }

    myrotation(int32_t index, uint32_t rate, bool reverse = false) : rotation(index, reverse) {
        datarate(rate);       
    }

    // min is 5ms
    void datarate(uint32_t rate)
    {
        rotation::datarate(rate);
    }

    double position(rotationUnits units)
    {
        return rotation::position(units) / (double) gear_ratio;
    }

    void gearratio(float ratio)
    {
        gear_ratio = ratio;
    }

};

class rollingBuffer {
public:

    struct tSensorSample {
        uint32_t time; // system time
        uint32_t timestamp; // sensor time
        float position; // position in turns
        float velocity; // velocity in rpm
    } *bufferData = NULL, resampledData[2];

    int size = 0;
    int head = 0;
    int count = 0;

    rollingBuffer(int size) {
        this->size = size;
        bufferData = new tSensorSample[size];
        head = 0;
    }

    ~rollingBuffer() {
        delete[] bufferData;
    }

    void clear() {
        head = 0;
        count = 0;
    }

    bool isEmpty() {
        return (count == 0);
    }

    void initialize(tSensorSample value) {
        for (int i = 0; i < size; i++) {
            memcpy(&(bufferData[i]), &value, sizeof(tSensorSample));
        }
        count = size;
        head = 0;
    }

    int push(tSensorSample value) {
        if (bufferData == NULL || size <= 0) return;
        if (isEmpty()) {
            initialize(value);
            resampledData[0] = value;
            resampledData[1] = value;
            return 1;
        }
        if (value.timestamp == bufferData[head].timestamp)  return 0;

        head = (head + 1) % size;
        memcpy(&(bufferData[head]), &value, sizeof(tSensorSample));

        return 1;
    }

    // 0 is most recent, 1 is previous, 2 is oldest, etc.
    // TODO: add version that returns positive version for iterators
    tSensorSample get(int index) {
        if (head - index < 0) index = head - index + size;
        else index = head - index;
        return bufferData[index];
    }

    // 0 is most recent, 1 is previous
    tSensorSample get_reampled(int index) {
        return resampledData[index];
    }

    void interpolate(uint32_t time)
    {
        if (bufferData == NULL || size <= 0) return;
        if (isEmpty()) return;
        if (count < 2) return; // not enough data to interpolate

        // 1 will contain the previous reasmpled value and 0 will contain the current one
        resampledData[1] = resampledData[0];

        tSensorSample latestSample = get(0);
        tSensorSample previousSample = get(1);

        if (time == latestSample.timestamp) {
            resampledData[0] = latestSample;
            return;
        }

        if (time == previousSample.timestamp) {
            resampledData[0] = previousSample;
            return;
        }

        float s0 = previousSample.position;
        float s1 = latestSample.position;
        float t0 = previousSample.timestamp;
        float t1 = latestSample.timestamp;
        float t_new = (float) time;

        if (t0 == t1) {
            resampledData[0] = latestSample;
            return;
        }

        float s_new = s0 + (t_new - t0) * (s1 - s0) / (t1 - t0);

        resampledData[0].timestamp = time;
        resampledData[0].position = s_new;

    }

};

// extern "C" void rotation_update_callback(void *obj) {
//     myrotation::GetLatestWrapper(obj);
// }
