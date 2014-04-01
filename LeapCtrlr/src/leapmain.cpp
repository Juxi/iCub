/******************************************************************************\
* Copyright (C) Leap Motion, Inc. 2011-2013.                                   *
* Leap Motion proprietary and  confidential.  Not for distribution.            *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement between *
* Leap Motion and you, your company or other organization.                     *
\******************************************************************************/
#include "../JuceLibraryCode/JuceHeader.h"
#include "Leap.h"
#include "LeapUtilGL.h"
#include <cctype>

class MotionVisualizerWindow;
class OpenGLCanvas;

static const float kfNumGrids   = 15.0f;
static const float kfGridScale  = 120.0f;

//==============================================================================
class MotionVisualizerApplication  : public JUCEApplication
{
public:
    //==============================================================================
    MotionVisualizerApplication()
    {
    }

    ~MotionVisualizerApplication()
    {
    }

    //==============================================================================
    void initialise (const String& commandLine);

    void shutdown()
    {
        // Do your application's shutdown code here..
        
    }

    //==============================================================================
    void systemRequestedQuit()
    {
        quit();
    }

    //==============================================================================
    const String getApplicationName()
    {
        return "Leap Motion Visualizer";
    }

    const String getApplicationVersion()
    {
        return ProjectInfo::versionString;
    }

    bool moreThanOneInstanceAllowed()
    {
        return true;
    }

    void anotherInstanceStarted (const String& commandLine)
    {
      (void)commandLine;        
    }

    static Leap::Controller& getController() 
    {
        static Leap::Controller s_controller;

        return  s_controller;
    }

private:
    ScopedPointer<MotionVisualizerWindow>  m_pMainWindow; 
};

//==============================================================================
class OpenGLCanvas  : public Component,
                      public OpenGLRenderer,
                      Leap::Listener
{
public:
    enum
    {
      kFlag_FPS         = 1 << 0,
      kFlag_Help        = 1 << 1,
      kFlag_RedBlue3D   = 1 << 2,
       
      kFlag_Translate   = 1 << 3,
      kFlag_Rotate      = 1 << 4,
      kFlag_Scale       = 1 << 5,
      kFlag_Automatic   = 1 << 6,

      kFlag_AutoDetectedTranslate   = 1 << 7,
      kFlag_AutoDetectedRotate      = 1 << 8,
      kFlag_AutoDetectedScale       = 1 << 9,
      kFlags_AutoDetectedTransforms = kFlag_AutoDetectedTranslate|kFlag_AutoDetectedRotate|kFlag_AutoDetectedScale,
    };

    enum eDrawMode
    {
      kDraw_Normal,
      kDraw_3DLeft,
      kDraw_3DRight
    };

    OpenGLCanvas()
      : Component( "OpenGLCanvas" )
    {
        m_openGLContext.setRenderer (this);
        m_openGLContext.setComponentPaintingEnabled (true);
        m_openGLContext.attachTo (*this);
        setBounds( 0, 0, 1024, 768 );

        m_fLastUpdateTimeSeconds = Time::highResolutionTicksToSeconds(Time::getHighResolutionTicks());
        m_fLastRenderTimeSeconds = m_fLastUpdateTimeSeconds;

        MotionVisualizerApplication::getController().addListener( *this );

        resetCamera();

        setWantsKeyboardFocus( true );

        m_uiFlags = kFlag_Rotate | kFlag_Scale | kFlag_Translate | kFlag_Automatic;

        updateStateStr();

        m_strPrompt = "Press 'H' for help";

        m_strHelp = "ESC - quit\n"
                    "H - Toggle help and frame rate display\n"
                    "A - Toggle automatic motion tracking\n"
                    "R - Toggle rotation tracking\n"
                    "S - Toggle scale tracking\n"
                    "T - Toggle translation tracking\n"
                    "3 - Toggle red/blue 3D mode\n"
                    "Page Up   - Increase left/right eye separation\n"
                    "Page Down - Decrease left/right eye separation\n"
                    "Space     - Reset the scene and camera";

        m_fTotalMotionScale = 1.0f;

        m_fLeftRightEyeShift = 2.0f;
    }

    ~OpenGLCanvas()
    {
        MotionVisualizerApplication::getController().removeListener( *this );
        m_openGLContext.detach();
    }

    void newOpenGLContextCreated()
    {
        glEnable(GL_BLEND);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(true);
        glDepthFunc(GL_LESS);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

        m_fixedFont = Font("Courier New", 24, Font::plain );
    }

    void openGLContextClosing()
    {
    }

    bool keyPressed( const KeyPress& keyPress )
    {
      int iKeyCode = toupper(keyPress.getKeyCode());

      if ( iKeyCode == KeyPress::escapeKey )
      {
        JUCEApplication::quit();
        return true;
      }

      if ( iKeyCode == KeyPress::pageUpKey )
      {
        float fStep = (m_fLeftRightEyeShift < 1.0f) ? 0.05f : ((m_fLeftRightEyeShift < 5.0f)  ? 0.25f : 1.0f);
        m_fLeftRightEyeShift = LeapUtil::Min( m_fLeftRightEyeShift + fStep, 50.0f );
        updateStateStr();
        return true;
      }

      if ( iKeyCode == KeyPress::pageDownKey )
      {
        float fStep = (m_fLeftRightEyeShift > 5.0f) ? 1.0f : ((m_fLeftRightEyeShift > 1.0f)  ? 0.25f : 0.05f);
        m_fLeftRightEyeShift = LeapUtil::Max( m_fLeftRightEyeShift - fStep, 0.0f );
        updateStateStr();
        return true;
      }

      switch ( iKeyCode )
      {
      case ' ':
        resetScene();
        resetCamera();
        return true;

      case '3':
        m_uiFlags ^= kFlag_RedBlue3D;
        break;

      case 'A':
        m_uiFlags ^= kFlag_Automatic;
        break;

      case 'H':
        m_uiFlags ^= (kFlag_Help | kFlag_FPS);
        return true;

      case 'R':
        m_uiFlags ^= kFlag_Rotate;
        break;

      case 'S':
        m_uiFlags ^= kFlag_Scale;
        break;

      case 'T':
        m_uiFlags ^= kFlag_Translate;
        break;

      default:
        return false;
      }

      updateStateStr();

      return true;
    }

    void updateStateStr()
    {
      m_aStrState[0] = String::empty;
      m_aStrState[1] = String::empty;
      m_aStrState[2] = String::empty;
      m_aStrState[3] = String::empty;
      m_aStrState[4] = String::empty;

      m_aStrState[0]  << "[T]ranslate: " << LeapUtil::BoolToStr( m_uiFlags & kFlag_Translate );
      m_aStrState[1]  << "[R]otate: "    << LeapUtil::BoolToStr( m_uiFlags & kFlag_Rotate );
      m_aStrState[2]  << "[S]cale: "     << LeapUtil::BoolToStr( m_uiFlags & kFlag_Scale );
      m_aStrState[3]  << "[A]utomatic: " << LeapUtil::BoolToStr( m_uiFlags & kFlag_Automatic );

      if ( m_uiFlags & kFlag_RedBlue3D )
      {
        m_aStrState[4] << "Left/Right Shift: " << String(m_fLeftRightEyeShift, 2);
      }
    }

    void mouseDown (const MouseEvent& e)
    {
      (void)e;
    }

    void mouseDrag (const MouseEvent& e)
    {
      (void)e;
    }

    void mouseWheelMove ( const MouseEvent& e,
                          const MouseWheelDetails& wheel )
    {
      (void)e;
      (void)wheel;
    }

    void resized()
    {
    }

    void paint(Graphics& g)
    {
      (void)g;
    }

    void renderOpenGL2D() 
    {
        LeapUtilGL::GLAttribScope attribScope( GL_ENABLE_BIT );

        // when enabled text draws poorly.
        glDisable(GL_CULL_FACE);

        ScopedPointer<LowLevelGraphicsContext> glRenderer (createOpenGLGraphicsContext (m_openGLContext, getWidth(), getHeight()));

        if (glRenderer == nullptr)
        {
          return;
        }

        Graphics g(*glRenderer.get());

        int iMargin   = 10;
        int iFontSize = static_cast<int>(m_fixedFont.getHeight()); 
        int iLineStep = iFontSize + (iFontSize >> 2);
        int iBaseLine = 20;
        Font origFont = g.getCurrentFont();

        const Rectangle<int>& rectBounds = getBounds();

        g.setFont( static_cast<float>(iFontSize) );

        g.setColour( Colours::salmon );
        g.drawSingleLineText( m_strPrompt, iMargin, iBaseLine );

        if ( m_uiFlags & kFlag_FPS )
        {
          g.setColour( Colours::seagreen );
          g.drawSingleLineText( m_strUpdateFPS, iMargin, iBaseLine + iLineStep );
          g.drawSingleLineText( m_strRenderFPS, iMargin, iBaseLine + iLineStep*2 );
        }

        if ( m_uiFlags & kFlag_Help )
        {
          g.setFont( m_fixedFont );
          g.setColour( Colours::slateblue );
          g.drawMultiLineText( m_strHelp,
                               iMargin,
                               iBaseLine + iLineStep * 3,
                               rectBounds.getWidth() - iMargin*2 );
        }

        g.setFont( origFont );
        g.setFont( static_cast<float>(iFontSize) );

        g.setColour( Colours::orange );

        for ( int i = 4, uiTotalWidth = 0; i >= 0; i-- )
        {
          if ( !m_aStrState[i].isEmpty() )
          {
            const uint32_t uiStrWidth = g.getCurrentFont().getStringWidth( m_aStrState[i] );

            if ( (i < 3) && (m_uiFlags & kFlag_Automatic) ) 
            {
              const uint32_t uiXformFlags = (kFlag_Translate << i)|(kFlag_AutoDetectedTranslate << i);
              const bool bDidTransform =  (m_uiFlags & uiXformFlags) == uiXformFlags;                                    
              g.setColour( bDidTransform ? Colours::gold : Colours::grey );
            }

            g.drawSingleLineText( m_aStrState[i],
                                  rectBounds.getRight() - iMargin - uiTotalWidth, 
                                  rectBounds.getBottom() - (iFontSize*2),
                                  Justification::right );

            uiTotalWidth += uiStrWidth + iFontSize;
          }
        }
    }

    //
    // calculations that should only be done once per leap data frame but may be drawn many times should go here.
    //   
    void update( Leap::Frame frame )
    {
        ScopedLock sceneGraphLock(m_renderMutex);

        double curSysTimeSeconds = Time::highResolutionTicksToSeconds(Time::getHighResolutionTicks());

        float deltaTimeSeconds = static_cast<float>(curSysTimeSeconds - m_fLastUpdateTimeSeconds);
      
        m_fLastUpdateTimeSeconds = curSysTimeSeconds;

        float fUpdateDT  = m_avgUpdateDeltaTime.AddSample( deltaTimeSeconds );
        float fUpdateFPS = (fUpdateDT > 0) ? 1.0f/fUpdateDT : 0.0f;
        m_strUpdateFPS = String::formatted( "UpdateFPS: %4.2f", fUpdateFPS );

        static const float kfMinScale = 0.1f;

        static const float kfMaxScale = 2.0f;

        //If automatic motion type detection is enabled, check if motion probabilities are above threshold amounts
        bool bShouldTranslate = true;
        bool bShouldRotate    = true;
        bool bShouldScale     = true;

        m_uiFlags &= ~(kFlags_AutoDetectedTransforms);

        if ( m_uiFlags & kFlag_Automatic )
        {
          bShouldTranslate = frame.translationProbability(m_lastFrame) > 0.40;
          bShouldRotate    = frame.rotationProbability(m_lastFrame)    > 0.40;
          bShouldScale     = frame.scaleProbability(m_lastFrame)       > 0.40;

          m_uiFlags |= bShouldTranslate ? kFlag_AutoDetectedTranslate : 0;
          m_uiFlags |= bShouldRotate ? kFlag_AutoDetectedRotate : 0;
          m_uiFlags |= bShouldScale ? kFlag_AutoDetectedScale : 0;
        }

        //Update the translation in the current reference frame
        if (( m_uiFlags & kFlag_Translate ) && bShouldTranslate)
        {
          m_vTotalMotionTranslation += m_mtxTotalMotionRotation.rigidInverse().transformDirection(frame.translation(m_lastFrame));
        }

        //Update the rotation
        if (( m_uiFlags & kFlag_Rotate ) && bShouldRotate)
        {
          m_mtxTotalMotionRotation = frame.rotationMatrix(m_lastFrame) * m_mtxTotalMotionRotation; // Left multiply, relative to last frame
        }

        //Update the scale
        if (( m_uiFlags & kFlag_Scale ) && bShouldScale)
        {
          m_fTotalMotionScale = LeapUtil::Clamp(  m_fTotalMotionScale * frame.scaleFactor(m_lastFrame),
                                                  kfMinScale,
                                                  kfMaxScale );
        }

        //Shift grid translation if too far to a side
        if (m_vTotalMotionTranslation.x <= -kfGridScale) 
        {
          m_vTotalMotionTranslation.x += kfGridScale;
        } 
        else if (m_vTotalMotionTranslation.x >= kfGridScale) 
        {
          m_vTotalMotionTranslation.x -= kfGridScale;
        }

        if (m_vTotalMotionTranslation.y <= -kfGridScale) 
        {
          m_vTotalMotionTranslation.y += kfGridScale;
        } 
        else if (m_vTotalMotionTranslation.y >= kfGridScale) 
        {
          m_vTotalMotionTranslation.y -= kfGridScale;
        }

        if (m_vTotalMotionTranslation.z <= -kfGridScale) 
        {
          m_vTotalMotionTranslation.z += kfGridScale;
        } 
        else if (m_vTotalMotionTranslation.z >= kfGridScale) 
        {
          m_vTotalMotionTranslation.z -= kfGridScale;
        }
    }

    /// affects model view matrix.  needs to be inside a glPush/glPop matrix block!
    void setupScene()
    {
        OpenGLHelpers::clear( Colours::black.withAlpha(0.0f) );

        m_camera.SetAspectRatio( getWidth() / static_cast<float>(getHeight()) );

        m_camera.SetupGLProjection();

        m_camera.SetupGLView();

        /// JUCE turns off the depth test every frame when calling paint.
        glEnable(GL_DEPTH_TEST);
        glDepthMask(true);
        glDepthFunc(GL_LESS);

        glEnable(GL_BLEND);
        glDisable(GL_LIGHTING);
    }

    void drawScene( eDrawMode drawMode )
    {
        LeapUtilGL::GLMatrixScope sceneMatrixScope;

        Leap::Vector vColor( 1, 1, 1 );

        switch ( drawMode )
        {
        case kDraw_Normal:
          // don't need to do anything
          break;
        case kDraw_3DLeft:
          {
            Leap::Vector vTrans = m_camera.GetPOV().xBasis * m_fLeftRightEyeShift * m_fTotalMotionScale;

            glTranslatef( vTrans.x, vTrans.y, vTrans.z );

            // left eye, draw in red.
            vColor = Leap::Vector( 1, 0, 0 );
          }
          break;
        case kDraw_3DRight:
          {
            Leap::Vector vTrans = m_camera.GetPOV().xBasis * -m_fLeftRightEyeShift * m_fTotalMotionScale;

            glTranslatef( vTrans.x, vTrans.y, vTrans.z );

            // right eye draw in blue
            vColor = Leap::Vector( 0, 0, 1 );
          }
          break;
        }

        //Set the 3D grid transformation matrix
        glMultMatrixf(m_mtxTotalMotionRotation.toArray4x4());
        glScalef(m_fTotalMotionScale, m_fTotalMotionScale, m_fTotalMotionScale);
        glTranslatef(m_vTotalMotionTranslation.x, m_vTotalMotionTranslation.y, m_vTotalMotionTranslation.z);

        //Draw the infinite grid
        static const float kfSide = kfNumGrids*kfGridScale*0.5f;
        static const float kfAtten = kfGridScale*kfGridScale;

        glLineWidth(1.0f);
        glBegin(GL_LINES);

        for ( float i = -kfSide; i < kfSide; i += kfGridScale) 
        {
          for ( float j = -kfSide; j < kfSide; j += kfGridScale) 
          {
            for ( float k = -kfSide; k < kfSide; k += kfGridScale) 
            {
              const float fIntens1 = kfAtten/((m_vTotalMotionTranslation + Leap::Vector(i, j, k)).magnitudeSquared() + kfAtten);
              const float fIntens2 = kfAtten/((m_vTotalMotionTranslation + Leap::Vector(i+kfGridScale, j, k)).magnitudeSquared() + kfAtten);
              const float fIntens3 = kfAtten/((m_vTotalMotionTranslation + Leap::Vector(i, j+kfGridScale, k)).magnitudeSquared() + kfAtten);
              const float fIntens4 = kfAtten/((m_vTotalMotionTranslation + Leap::Vector(i, j, k+kfGridScale)).magnitudeSquared() + kfAtten);

              const Leap::Vector vColor1 = vColor * fIntens1;
              const Leap::Vector vColor2 = vColor * fIntens2;
              const Leap::Vector vColor3 = vColor * fIntens3;
              const Leap::Vector vColor4 = vColor * fIntens4;

              glColor3fv( vColor1.toFloatPointer() );
              glVertex3f(i, j, k);

              glColor3fv( vColor2.toFloatPointer() );
              glVertex3f(i + kfGridScale, j, k);

              glColor3fv( vColor1.toFloatPointer() );
              glVertex3f(i, j, k);

              glColor3fv( vColor3.toFloatPointer() );
              glVertex3f(i, j + kfGridScale, k);

              glColor3fv( vColor1.toFloatPointer() );
              glVertex3f(i, j, k);

              glColor3fv( vColor4.toFloatPointer() );
              glVertex3f(i, j, k + kfGridScale);
            }
          }
        }

        glEnd();
    }

    // data should be drawn here but no heavy calculations done.
    // any major calculations that only need to be updated per leap data frame
    // should be handled in update and cached in members.
    void renderOpenGL()
    {
        {
          MessageManagerLock mm (Thread::getCurrentThread());
          if (! mm.lockWasGained())
            return;
        }

        Leap::Frame frame = m_lastFrame;

        double  fCurSysTimeSeconds = Time::highResolutionTicksToSeconds(Time::getHighResolutionTicks());
        float   fRenderDT = static_cast<float>(fCurSysTimeSeconds - m_fLastRenderTimeSeconds);
        fRenderDT = m_avgRenderDeltaTime.AddSample( fRenderDT );

        m_fLastRenderTimeSeconds = fCurSysTimeSeconds;

        float fRenderFPS = (fRenderDT > 0) ? 1.0f/fRenderDT : 0.0f;

        m_strRenderFPS = String::formatted( "RenderFPS: %4.2f", fRenderFPS );

        LeapUtilGL::GLMatrixScope sceneMatrixScope;

        setupScene();

        {
          LeapUtilGL::GLAttribScope sceneAttribScope( GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT );

          if ( m_uiFlags & kFlag_RedBlue3D )
          {
            glEnable(GL_BLEND);
            glDisable(GL_DEPTH_TEST);
            glBlendFunc(GL_ONE, GL_ONE);
            drawScene( kDraw_3DLeft );
            drawScene( kDraw_3DRight );
          }
          else
          {
            drawScene( kDraw_Normal );
          }
        }

        {
          float fBottomPos      = -0.35f;
          float fLeftPos        = fBottomPos * m_camera.GetAspectRatio();
          Leap::Vector vAxisPos = m_camera.GetPOV().transformPoint( Leap::Vector( fLeftPos, fBottomPos, -1.0f ) );

          // draw the frame transform orientation axes
          glTranslatef( vAxisPos.x, vAxisPos.y, vAxisPos.z );
          glMultMatrixf( m_mtxTotalMotionRotation.toArray4x4() );
          glScalef( 0.05f, 0.05f, 0.05f );

          LeapUtilGL::drawAxes();
        }

        {
          ScopedLock renderLock(m_renderMutex);
          
          // draw the text overlay
          renderOpenGL2D();
        }
    }

    virtual void onInit(const Leap::Controller&) 
    {
    }

    virtual void onConnect(const Leap::Controller&) 
    {
    }

    virtual void onDisconnect(const Leap::Controller&) 
    {
    }

    virtual void onFocusGained(const Leap::Controller& controller)
    {
      m_lastFrame = controller.frame();
    }

    virtual void onFocusLost(const Leap::Controller&)
    {
    }

    virtual void onFrame(const Leap::Controller& controller)
    {
        Leap::Frame frame = controller.frame();

        update( frame );
        m_lastFrame = frame;
        m_openGLContext.triggerRepaint();
    }

    void resetScene()
    {
        m_mtxTotalMotionRotation  = Leap::Matrix::identity();
        m_vTotalMotionTranslation = Leap::Vector::zero();
        m_fTotalMotionScale       = 1.0f;
    }

    void resetCamera()
    {
        m_camera.SetVerticalFOVDegrees( 45.0f );
        m_camera.SetClipPlanes( 0.1f, 2000.0f );
        m_camera.SetOrbitTarget( Leap::Vector::zero() );
        m_camera.SetPOVLookAt( Leap::Vector( 0, 0, 120 ), m_camera.GetOrbitTarget() );
    }

private:
    OpenGLContext               m_openGLContext;
    LeapUtilGL::CameraGL        m_camera;
    Leap::Frame                 m_lastFrame;
    double                      m_fLastUpdateTimeSeconds;
    double                      m_fLastRenderTimeSeconds;

    /// accumulated rotation/translation and scale from Leap Motion API
    Leap::Matrix                m_mtxTotalMotionRotation;
    Leap::Vector                m_vTotalMotionTranslation;
    float                       m_fTotalMotionScale;
    float                       m_fLeftRightEyeShift;

    Font                        m_fixedFont;
    LeapUtil::RollingAverage<>  m_avgUpdateDeltaTime;
    LeapUtil::RollingAverage<>  m_avgRenderDeltaTime;
    String                      m_strUpdateFPS;
    String                      m_strRenderFPS;
    String                      m_strHelp;
    String                      m_aStrState[5];
    String                      m_strPrompt;
    uint32_t                    m_uiFlags;
    CriticalSection             m_renderMutex;
};

//==============================================================================
/**
    This is the top-level window that we'll pop up. Inside it, we'll create and
    show a component from the MainComponent.cpp file (you can open this file using
    the Jucer to edit it).
*/
class MotionVisualizerWindow : public DocumentWindow
{
public:
    //==============================================================================
    MotionVisualizerWindow()
        : DocumentWindow ("Leap Motion Visualizer",
                          Colours::lightgrey,
                          DocumentWindow::allButtons,
                          true)
    {
        setContentOwned (new OpenGLCanvas(), true);

        // Centre the window on the screen
        centreWithSize (getWidth(), getHeight());

        // And show it!
        setVisible (true);

        getChildComponent(0)->grabKeyboardFocus();
    }

    ~MotionVisualizerWindow()
    {
        // (the content component will be deleted automatically, so no need to do it here)
    }

    //==============================================================================
    void closeButtonPressed()
    {
        // When the user presses the close button, we'll tell the app to quit. This
        JUCEApplication::quit();
    }
};

void MotionVisualizerApplication::initialise (const String& commandLine)
{
    (void) commandLine;
    // Do your application's initialisation code here..
    m_pMainWindow = new MotionVisualizerWindow();
}

//==============================================================================
// This macro generates the main() routine that starts the app.
START_JUCE_APPLICATION(MotionVisualizerApplication)
