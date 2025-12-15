#pragma once

template <class item_t, int num_items = 64>
class Queue
{
public:
  Queue()
  {
    clear();
  }

  int size()
  {
    return mItemCount;
  }

  void clear()
  {
    mHead = 0;
    mTail = 0;
    mItemCount = 0;
  }

  void add(item_t item)
  {
    if (mItemCount >= num_items)
    {

      return;
    }
    mData[mTail] = item;
    ++mTail;
    ++mItemCount;
    if (mTail >= num_items + 1)
    {
      mTail = 0;
    }
  }

  item_t head()
  {
    if (mItemCount == 0)
    {

      return item_t();
    }
    item_t result = mData[mHead];
    ++mHead;
    if (mHead >= num_items + 1)
    {
      mHead = 0;
    }
    --mItemCount;
    return result;
  }

protected:
  item_t mData[num_items + 1];
  int mHead = 0;
  int mTail = 0;
  int mItemCount = 0;

private:
  Queue(const Queue<item_t> &rhs)
  {
  }
};