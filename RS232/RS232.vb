'Serial Port Interfacing with VB.net 2010 Express Edition
'Copyright (C) 2011  Dev Shrestha


Imports System.ComponentModel

'Imports Microsoft.VisualBasic.FileIO


Public Class frmRS232

    Dim myPort As Array  'COM Ports detected on the system will be stored here
    Dim Sentence As String
    Private _Connect As Boolean = True
    Private _OpenForms As Integer = 0
    Private _R2323s() As frmRS232
    Delegate Sub SetTextCallback(ByVal [text] As String) 'Added to prevent threading errors during receiveing of data

    Private Sub frmMain_Load(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles MyBase.Load

        Port_Refresh()
        cb_lineender.Items.Clear()
        cb_lineender.Items.Add("None")
        cb_lineender.Items.Add("Cr")
        cb_lineender.Items.Add("Lf")
        cb_lineender.Items.Add("CrLf")
        cb_lineender.SelectedIndex = 0

        Try

            cb_lineender.Text = cmbPort.Items.Item(My.Settings.LineEnder)    'Set cmbPort text to the first COM port detected

        Catch
            cb_lineender.Text = "None"

        End Try

    End Sub

    Private Sub btnConnect_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnConnect.Click
        If (_Connect) Then
            My.Settings.Port = cmbPort.SelectedIndex
            My.Settings.Baud = cmbBaud.SelectedIndex
            My.Settings.Save()

            SerialPort1.PortName = cmbPort.Text         'Set SerialPort1 to the selected COM port at startup
            SerialPort1.BaudRate = cmbBaud.Text         'Set Baud rate to the selected value on 

            'Other Serial Port Property
            SerialPort1.Parity = IO.Ports.Parity.None
            SerialPort1.StopBits = IO.Ports.StopBits.One
            SerialPort1.DataBits = 8            'Open our serial port
            SerialPort1.Open()
            _Connect = False
            btnConnect.Text = "Disconnect"
            'btnConnect.Enabled = False          'Disable Connect button
            'btnDisconnect.Enabled = True        'and Enable Disconnect button
        Else
            SerialPort1.Close()             'Close our Serial Port

            'btnConnect.Enabled = True
            'btnDisconnect.Enabled = False
            _Connect = True
            btnConnect.Text = "Connect"
        End If

    End Sub

    Private Sub btnRefresh_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnRefresh.Click
        Port_Refresh()

    End Sub



    Private Sub SerialPort1_DataReceived(ByVal sender As Object, ByVal e As System.IO.Ports.SerialDataReceivedEventArgs) Handles SerialPort1.DataReceived
        ReceivedText(SerialPort1.ReadExisting())    'Automatically called every time a data is received at the serialPort
    End Sub
    Private Sub ReceivedText(ByVal [text] As String)
        ' This function can be called multiple times per line
        'compares the ID of the creating Thread to the ID of the calling Thread
        'Dim TimeStamp As String
        If Me.rtbReceived.InvokeRequired Then
            Dim x As New SetTextCallback(AddressOf ReceivedText)
            Me.Invoke(x, New Object() {(text)})
        Else
            'TimeStamp = Format(Now(), "mm/dd/yyyy hh:mm:ss") + vbTab
            'Sentence &= [text]
            rtbReceived.Text &= [text]
            rtbReceived.SelectionStart = rtbReceived.Text.Length
            rtbReceived.ScrollToCaret()
            'Fields = Sentence.Split(vbCrLf)

            'If UBound(Fields) > 0 Then ' if there is more than one sentences
            'Parse(Fields(0))
            'Sentence = LTrim(Fields(UBound(Fields)))

            'End If

        End If
    End Sub

    Private Sub cmbPort_SelectedIndexChanged(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles cmbPort.SelectedIndexChanged
        If SerialPort1.IsOpen = False Then
            SerialPort1.PortName = cmbPort.Text         'pop a message box to user if he is changing ports
        Else                                            'without disconnecting first.
            MsgBox("Valid only if port is Closed", vbCritical)
        End If
    End Sub

    Private Sub cmbBaud_SelectedIndexChanged(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles cmbBaud.SelectedIndexChanged
        If SerialPort1.IsOpen = False Then
            SerialPort1.BaudRate = cmbBaud.Text         'pop a message box to user if he is changing baud rate
        Else                                            'without disconnecting first.
            MsgBox("Valid only if port is Closed", vbCritical)
        End If
    End Sub
    ' This function attempts to open the passed Comm Port. If it is
    '   available, it returns True, else it returns False. To determine
    '   availability a Try-Catch block is used.
    Private Function IsPortAvailable(ByVal ComPort As Integer) As Boolean

        Try
            SerialPort1.PortName = myPort(ComPort)         'Set SerialPort1 to the selected COM port at startup
            SerialPort1.BaudRate = 9600         'Set Baud rate to the selected value on 

            'Other Serial Port Property
            SerialPort1.Parity = IO.Ports.Parity.None
            SerialPort1.StopBits = IO.Ports.StopBits.One
            SerialPort1.DataBits = 8            'Open our serial port
            SerialPort1.Open()

            ' If it makes it to here, then the Comm Port is available.
            SerialPort1.Close()
            Return True
        Catch
            ' If it gets here, then the attempt to open the Comm Port
            '   was unsuccessful.
            Return False
        End Try
    End Function




    Private Sub btnSave_Click(sender As Object, e As EventArgs) Handles btnSave.Click
        Dim File As String = txtFileFolder.Text
        If Dir(File) <> "" Then
            If (MsgBox("File exists." + vbCrLf + "Replace?", vbYesNo + vbCritical + 256, "Confirm") = vbYes) Then
                Dim out As IO.StreamWriter = My.Computer.FileSystem.OpenTextFileWriter(File, False)
                out.Write(rtbReceived.Text)
                out.Close()
            End If
        Else
            Dim out As IO.StreamWriter = My.Computer.FileSystem.OpenTextFileWriter(File, False)
            out.Write(rtbReceived.Text)
            out.Close()
        End If

    End Sub

    Private Sub btnClear_Click(sender As Object, e As EventArgs) Handles btnClear.Click
        Sentence = ""
        rtbReceived.Text = ""
    End Sub



    Sub Port_Refresh()
        'When our form loads, auto detect all serial ports in the system and populate the cmbPort Combo box.

        cmbBaud.Items.Clear()
        cmbPort.Items.Clear()
        myPort = IO.Ports.SerialPort.GetPortNames() 'Get all com ports available

        cmbBaud.Items.Add(9600)     'Populate the cmbBaud Combo box to common baud rates used   
        cmbBaud.Items.Add(19200)
        cmbBaud.Items.Add(38400)
        cmbBaud.Items.Add(57600) 'For Wintec
        cmbBaud.Items.Add(115200)
        cmbBaud.Items.Add(230400)

        For i = 0 To UBound(myPort)
            If IsPortAvailable(i) Then cmbPort.Items.Add(myPort(i))
        Next

        Try

            cmbPort.Text = cmbPort.Items.Item(My.Settings.Port)    'Set cmbPort text to the first COM port detected
            cmbBaud.Text = cmbBaud.Items.Item(My.Settings.Baud)    'Set cmbBaud text to the first Baud rate on the list
        Catch
            If (cmbPort.Items.Count > 0) Then
                cmbPort.Text = cmbPort.Items(cmbPort.Items.Count - 1)
                cmbBaud.Text = 9600
            Else
                cmbPort.Text = "No avaialbe port"
                cmbBaud.Text = 9600
            End If

        End Try


        'btnRefresh.Enabled = False           'Initially Disconnect Button is Disabled
    End Sub

    Private Sub btnSend_Click(sender As Object, e As EventArgs) Handles Button_Send.Click
        Dim TxtOut = TextBox1.Text
        If (cb_lineender.SelectedIndex = 1) Then
            TxtOut += vbCr
        ElseIf (cb_lineender.SelectedIndex = 1) Then
            TxtOut += vbLf
        ElseIf (cb_lineender.SelectedIndex = 2) Then
            TxtOut += vbCrLf
        End If
        If SerialPort1.IsOpen Then
            SerialPort1.Write(TxtOut)
        Else
            MsgBox("Serial port not open", vbCritical)
        End If

    End Sub

    Private Sub Btn_AddForm_Click(sender As Object, e As EventArgs) Handles Btn_AddForm.Click
        _OpenForms += 1
        ReDim _R2323s(_OpenForms)
        _R2323s(_OpenForms) = New frmRS232
        _R2323s(_OpenForms).Show()
    End Sub

    Private Sub frmRS232_Closing(sender As Object, e As CancelEventArgs) Handles Me.Closing
        If (Me._Connect = False) Then
            Me.SerialPort1.Close()
        End If
    End Sub

    Private Sub TextBox1_KeyPress(sender As Object, e As KeyPressEventArgs) Handles TextBox1.KeyPress

        If (e.KeyChar = vbCr) Then
            Me.Button_Send.PerformClick()
        End If
    End Sub


End Class
