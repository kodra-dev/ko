function void setCaptureWeight(float data[]; int indices[]; int joint_index; float value)
{
   int ci = find(indices, joint_index);
   if (ci < 0)
   {
      append(indices, joint_index);
      append(data, value);
   }
   else
   {
      data[ci] = value;
   }
}

function void setCaptureWeight(float data[]; int indices[]; string joints[]; string joint_name; float value)
{
    int ji = find(joints, joint_name);
    if (ji < 0)
    {
        append(joints, joint_name);
        ji = len(joints) - 1;
    }
    setCaptureWeight(data, indices, ji, value);
}

function float getCaptureWeight(float data[]; int indices[]; int joint_index)
{
   int ci = find(indices, joint_index);
   if (ci < 0)
   {
      return 0;
   }
   else
   {
      return data[ci];
   }
}

function float getCaptureWeight(float data[]; int indices[]; string joints[]; string joint_name)
{
    int ji = find(joints, joint_name);
    if (ji < 0)
    {
        return 0;
    }
    return getCaptureWeight(data, indices, ji);
}