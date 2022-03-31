using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;

namespace QuadClient
{
    public partial class QuadApp : Form
    {
        SerialPort uart = new SerialPort();
        private const byte CMD_get = 0x01;
        private const byte CMD_set = 0x02;
        private const byte CMD_joints = 0x10;
        private const byte CMD_positions = 0x20;
        private const byte CMD_offsets = 0x40;

        public QuadApp()
        {
            InitializeComponent();

            tb_rfb.Text = 0.ToString();
            tb_rfs.Text = 0.ToString();
            tb_rfk.Text = 0.ToString();
            tb_lfb.Text = 0.ToString();
            tb_lfs.Text = 0.ToString();
            tb_lfk.Text = 0.ToString();
            tb_rbb.Text = 0.ToString();
            tb_rbs.Text = 0.ToString();
            tb_rbk.Text = 0.ToString();
            tb_lbb.Text = 0.ToString();
            tb_lbs.Text = 0.ToString();
            tb_lbk.Text = 0.ToString();

            var ports = SerialPort.GetPortNames();
            foreach (var s in ports)
                cb_ports.Items.Add(s);
            if (ports.Length > 0)
                cb_ports.SelectedIndex = 0;
            uart.StopBits = StopBits.One;
            uart.Parity = Parity.None;
            uart.DataBits = 8;
            uart.BaudRate = 115200;
            uart.DataReceived += Uart_DataReceived;

            cb_setValues.Items.Add("Joints");
            cb_setValues.Items.Add("Positions");
            cb_setValues.Items.Add("Offsets");
            cb_setValues.SelectedIndex = 0;
        }

        private delegate void RecvDataWriting();
        
        private void UartRecvCallback()
        {
            switch (((byte)uart.ReadByte() & 0x0f))
            {
                case CMD_set:
                    break;
                case CMD_get:
                    byte[] vals = new byte[12*4];
                    uart.Read(vals, 0, vals.Length);
                    tb_rfb.Text = BitConverter.ToSingle(vals, 0).ToString();
                    tb_rfs.Text = BitConverter.ToSingle(vals, 4).ToString();
                    tb_rfk.Text = BitConverter.ToSingle(vals, 8).ToString();
                    tb_lfb.Text = BitConverter.ToSingle(vals, 12).ToString();
                    tb_lfs.Text = BitConverter.ToSingle(vals, 16).ToString();
                    tb_lfk.Text = BitConverter.ToSingle(vals, 20).ToString();
                    tb_rbb.Text = BitConverter.ToSingle(vals, 24).ToString();
                    tb_rbs.Text = BitConverter.ToSingle(vals, 28).ToString();
                    tb_rbk.Text = BitConverter.ToSingle(vals, 32).ToString();
                    tb_lbb.Text = BitConverter.ToSingle(vals, 36).ToString();
                    tb_lbs.Text = BitConverter.ToSingle(vals, 40).ToString();
                    tb_lbk.Text = BitConverter.ToSingle(vals, 44).ToString();
                    break;
            }
        }

        private void Uart_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (InvokeRequired)
                Invoke(new RecvDataWriting(UartRecvCallback));
            else
                UartRecvCallback();
        }

        private void btn_send_Click(object sender, EventArgs e)
        {
            try
            {
                float[] vals = new float[12];
                vals[0] = float.Parse(tb_rfb.Text);
                vals[1] = float.Parse(tb_rfs.Text);
                vals[2] = float.Parse(tb_rfk.Text);
                vals[3] = float.Parse(tb_lfb.Text);
                vals[4] = float.Parse(tb_lfs.Text);
                vals[5] = float.Parse(tb_lfk.Text);
                vals[6] = float.Parse(tb_rbb.Text);
                vals[7] = float.Parse(tb_rbs.Text);
                vals[8] = float.Parse(tb_rbk.Text);
                vals[9] = float.Parse(tb_lbb.Text);
                vals[10] = float.Parse(tb_lbs.Text);
                vals[11] = float.Parse(tb_lbk.Text);
                byte[] b = new byte[] { CMD_set };
                switch (cb_setValues.Text)
                {
                    case "Joints":
                        b[0] |= CMD_joints;
                        break;
                    case "Positions":
                        b[0] |= CMD_positions;
                        break;
                    case "Offsets":
                        b[0] |= CMD_offsets;
                        break;
                }
                uart.Write(b, 0, b.Length);
                foreach (var f in vals)
                {
                    b = BitConverter.GetBytes(f);
                    uart.Write(b, 0, b.Length);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void btn_recv_Click(object sender, EventArgs e)
        {
            try
            {
                byte[] b = new byte[] { CMD_get };
                switch (cb_setValues.Text)
                {
                    case "Joints":
                        b[0] |= CMD_joints;
                        break;
                    case "Positions":
                        b[0] |= CMD_positions;
                        break;
                    case "Offsets":
                        b[0] |= CMD_offsets;
                        break;
                }
                uart.Write(b, 0, b.Length);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void btn_connect_Click(object sender, EventArgs e)
        {
            try
            {
                if (btn_connect.Text == "Connect")
                {
                    uart.PortName = cb_ports.Text;
                    uart.Open();
                    btn_connect.Text = "Disconnect";
                }
                else
                {
                    uart.Close();
                    btn_connect.Text = "Connect";
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void cb_ports_DropDown(object sender, EventArgs e)
        {
            cb_ports.Items.Clear();
            foreach (var s in SerialPort.GetPortNames())
                cb_ports.Items.Add(s);
        }
    }
}
