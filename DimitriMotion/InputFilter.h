#ifndef INPUT_FILTER_H
#define INPUT_FILTER_H

class InputFilter {
public:
    InputFilter(int numSamples) : numSamples(numSamples), currentIndex(0), sum(0) {
        samples = new bool[numSamples]();
    }

    ~InputFilter() {
        delete[] samples;
    }

    bool filter(bool input) {
        // Update the sum by subtracting the oldest sample and adding the new input
        sum -= samples[currentIndex];
        sum += input;

        // Update the sample at the current index
        samples[currentIndex] = input;

        // Move to the next index
        currentIndex = (currentIndex + 1) % numSamples;

        // Calculate the average
        float average = (float)sum / numSamples;

        // Clamp the average value between 0 and 1
        average = (average < 0.0f) ? 0.0f : (average > 1.0f) ? 1.0f : average;

        return average == 1.0f;
    }

private:
    int numSamples;
    int currentIndex;
    int sum;
    bool* samples;
};

#endif // INPUT_FILTER_H
