#region License

/* Copyright (c) 2006 Leslie Sanford
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software. 
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 */

#endregion

#region Contact

/*
 * Leslie Sanford
 * Email: jabberdabber@hotmail.com
 */

#endregion

using System;
using System.ComponentModel;

namespace Multimedia
{
	/// <summary>
	/// Represents the basic functionality for all multimedia devices.
	/// </summary>
	public interface IDevice : IDisposable
	{
        /// <summary>
        /// Occurs when the IDevice has finished using a buffer.
        /// </summary>
        event EventHandler<BufferFinishedEventArgs> BufferFinished;

        /// <summary>
        /// Closes the IDevice.
        /// </summary>
        void Close();

        /// <summary>
        /// Resets the IDevice.
        /// </summary>
        void Reset();

        /// <summary>
        /// Gets the IDevice handle.
        /// </summary>
        int Handle
        {
            get;
        }

        /// <summary>
        /// Gets or sets the object used to marshal event-handler calls.
        /// </summary>
        ISynchronizeInvoke SynchronizingObject
        {
            get;
            set;
        }
	}

    /// <summary>
    /// Profides data for the BufferFinished event.
    /// </summary>
    public class BufferFinishedEventArgs : EventArgs
    {
        // The buffer that the IDevice is finished with.
        private byte[] buffer;

        /// <summary>
        /// Initializes a new instance of the BufferFinishedEventArgs class 
        /// with the specified buffer.
        /// </summary>
        /// <param name="buffer">
        /// The buffer that the IDevice is finished with.
        /// </param>
        public BufferFinishedEventArgs(byte[] buffer)
        {
            this.buffer = buffer;
        }

        /// <summary>
        /// Gets the buffer.
        /// </summary>
        /// <returns>
        /// The finished buffer.
        /// </returns>
        public byte[] GetBuffer()
        {
            return buffer;
        }
    }
}
