#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.guiincludes import *


class Camera_View(BufferedWindow):
    def __init__(self, parent, camera_name, *args, **kwargs):
        self.wxHelper = wxHelper.newInstance()
        #self.png = wx.Image(filepath, wx.BITMAP_TYPE_ANY)
        self.current_bitmap = self.wxHelper.BitmapFromImage(wx.Bitmap(512, 512))
        self.current_size = wx.Size(self.current_bitmap.GetWidth(), self.current_bitmap.GetHeight())
        BufferedWindow.__init__(self, parent, *args, **kwargs)
        self.SetMinSize(self.current_size)
        self.impil = None

        pub.subscribe(self.update_view, camera_name)
        #pub.subscribe(self.update_view, "update_camera1")
        self.UpdateDrawing()

    def update_view(self, msg):
        camera_msg = Camera_Message(msg)
        self.current_bitmap = self.to_bmp(camera_msg.np_image)
        size = wx.Size(self.current_bitmap.GetWidth(), self.current_bitmap.GetHeight())
        if size != self.current_size:
            self.current_size = wx.Size(self.current_bitmap.GetWidth(), self.current_bitmap.GetHeight())
            self.SetMinSize(self.current_size)

        rect = wx.Rect((self.ClientSize.x - self.current_size.x) / 2, (self.ClientSize.y - self.current_size.y) / 2,
                       self.current_size.x, self.current_size.y)
        self.UpdateDrawing(rect)

    def to_bmp(self, np_image):
        imcv = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        self.impil = Image.fromarray(imcv)
        imwx = self.wxHelper.CreateImage(self.impil.size[0], self.impil.size[1])
        imwx.SetData(self.impil.convert('RGB').tobytes())
        bitmap = self.wxHelper.BitmapFromImage(imwx)
        return self.scale_bitmap(bitmap)

    def scale_bitmap(self, bitmap):
        frame_size = self.ClientSize
        bitmap_size = bitmap.GetSize()
        if frame_size.x > 0 and frame_size.y > 0 and \
                (frame_size.x < bitmap_size.x or frame_size.y < bitmap_size.y):
            aspect = float(bitmap.GetWidth()) / float(bitmap.GetHeight())
            image = self.wxHelper.ImageFromBitmap(bitmap)
            frame_h = frame_size.y
            frame_w = frame_size.y * aspect
            image = image.Scale(frame_w, frame_h, wx.IMAGE_QUALITY_HIGH)
            scaled_bitmap = self.wxHelper.BitmapFromImage(image)
        else:
            scaled_bitmap = bitmap
        return scaled_bitmap

    def Draw(self, dc):
        dc.SetBackground(wx.Brush("White"))
        dc.Clear()

        if self.current_bitmap is not None:
            Size = self.ClientSize
            dc.DrawBitmap(self.current_bitmap,
                          max(0, (Size.x - self.current_size.x) / 2),
                          max(0, (Size.y - self.current_size.y) / 2))


