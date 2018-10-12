#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import wx


class wxHelper(object):
    def __init__(self, *args, **kwargs):
        pass

    @staticmethod
    def newInstance():
        if wx.__version__.startswith("4"):
            return wxHelper4()
        return wxHelper3()

    def BitmapFromImage(self, image):
        pass

    def CreateBitmap(self, size):
        pass

    def CreateImage(self, width, height):
        pass

    def ImageFromBitmap(self, bitmap):
        pass


class wxHelper4(wxHelper):
    def __init__(self, *args, **kwargs):
        super(wxHelper, self).__init__()

    def BitmapFromImage(self, image):
        return wx.Bitmap(image)

    def CreateBitmap(self, size):
        return wx.Bitmap(*size)

    def CreateImage(self, width, height):
        return wx.Image(width, height)

    def ImageFromBitmap(self, bitmap):
        return bitmap.ConvertToImage()


class wxHelper3(wxHelper):
    def __init__(self, *args, **kwargs):
        super(wxHelper, self).__init__()

    def BitmapFromImage(self, image):
        return wx.BitmapFromImage(image)

    def CreateBitmap(self, size):
        return wx.EmptyBitmap(*size)

    def CreateImage(self, width, height):
        return wx.EmptyImage(width, height)

    def ImageFromBitmap(self, bitmap):
        return wx.ImageFromBitmap(bitmap)
