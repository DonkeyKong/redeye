#pragma once

template <int NumSamples>
class MovingAverage
{
  float val_[NumSamples];
  int index_= -1;
public:
  void update(float val)
  {
    if (index_ == -1)
    {
      for (int i=0; i < NumSamples; ++i)
      {
        val_[i] = val;
      }
    }
    else
    {
      val_[index_] = val;
    }
    index_ = (index_ + 1) % NumSamples;
  }

  operator float() const
  {
    float avg = 0;
    for (int i=0; i < NumSamples; ++i)
    {
      avg += val_[i];
    }
    return avg / (float)NumSamples;
  }
};