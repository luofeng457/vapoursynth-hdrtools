# 1. HDRConvertYUV
1. `init()`

  ```cpp
  void HDRConvertYUV::init(ProjectParameters *inputParams)
  {
      // 1. init input/output frame format

      // 2. open input file
      IOFunctions::openFile(m_inputFile);

      // 3. init input frame
      m_inputFrame = Input::create(m_inputFile, input, inputParams);

      // 4. open output file
      IOFunctions::openFile(m_outputFile);

      // create frame memory as necessary Input.
      // This has the same format as the Input file.
      m_iFrameStore = new Frame();
      m_iFrameStore->clear();

      // if crop flags is set, create cropped frame store

      // 5. create output frame
      m_outputFrame = Output::create(m_outputFile, output);

      // 6. chroma format conversion if needed
      m_pFrameStore[0] = new Frame()

      // 7. Here we may convert to the output's format type
      // (e.g. from integer to float).
      m_convertFrameStore = new Frame()

      
      // 8. Also creation of frame store for the transfer function processed
      // images
      m_pFrameStore[3]  = new Frame(m_width, m_height, output->m_isFloat)

      // 9. Frame store for the inversion of PQ TF
      m_pFrameStore[1] = new Frame(m_width, m_height, TRUE)
      m_oFrameStore = new Frame()

      // 10. initiate the color transform process
      if (m_iFrameStore->m_colorSpace == CM_YCbCr || CM_ICtCp || others)

        m_colorTransform = ColorTransform::create()
        m_colorSpaceConvert = ColorTransform::create()

        frame store for color format conversion
        m_pFrameStore[2]  = new Frame()

        m_colorSpaceFrame = new Frame(m_width, m_height, TRUE)

      // 11. Chroma subsampling
      if (input->m_chromaFormat == CF_444 || CF_420
         m_convertFormatIn = ConvertColorFormat::create()
         m_convertFormatOut = ConvertColorFormat::create()


      // 12. what?
      m_pFrameStore[4] = new Frame(m_width, m_height)
      m_pFrameStore[6] = new Frame(m_width, m_height)

      // 13. Input/Output transfer function create
      m_inputTransferFunction = TransferFunction::create()
      m_outputTransferFunction = TransferFunction::create()

      // 14. Format conversion process
      m_convertIQuantize = Convert::create(&m_iFrameStore->m_format,
                                         &m_convertFrameStore->m_format);
      m_convertProcess = Convert::create(&m_pFrameStore[4]->m_format, output);
      if (m_bUseChromaDeblocking == TRUE)
          m_frameFilter = FrameFilter::create(m_width, m_height, FT_DEBLOCK);
  }
  ```

1. process()

  ```cpp
  void HDRConvertYUV::process(ProjectParameters *inputParams)
  {
      // 0. init
      float fDistance0 =
          inputParams->m_source.m_frameRate / inputParams->m_output.m_frameRate;
      // FrameFormat   *output = &inputParams->m_output;
      FrameFormat *input = &inputParams->m_source;
  
      Frame *currentFrame = NULL;
  
      // 1. Now process all frames
      for (frameNumber = 0; frameNumber < inputParams->m_numberOfFrames;
           frameNumber++) {
          iCurrentFrameToProcess = int(frameNumber * fDistance0);
  
          // 2. read frames
          if (m_inputFrame->readOneFrame() {
              // Now copy input frame buffer to processing frame buffer for any
              // subsequent processing
              m_inputFrame->copyFrame(m_iFrameStore);

          currentFrame = m_iFrameStore;
          if (m_croppedFrameStore != NULL) {
              m_croppedFrameStore->copy(m_iFrameStore);
              currentFrame = m_croppedFrameStore;
          }
  
          // 3. convert chroma format if needed
          m_convertIQuantize->process(m_pFrameStore[0], currentFrame);
          m_frameFilter->process(m_pFrameStore[0]);
          m_convertFormatIn->process(m_convertFrameStore, m_pFrameStore[0]);
  
          // 4. Add noise and filtering
          m_addNoise->process(m_convertFrameStore);
  
          if (m_bUseWienerFiltering == TRUE)
              m_frameFilterNoise0->process(m_convertFrameStore);
          if (m_bUse2DSepFiltering == TRUE)
              m_frameFilterNoise1->process(m_convertFrameStore);
          if (m_bUseNLMeansFiltering == TRUE)
              m_frameFilterNoise2->process(m_convertFrameStore);
  
          // 5. Now perform a color format conversion
          // Output to m_pFrameStore memory with appropriate color space
          // conversion
          // Note that the name of "forward" may be a bit of a misnomer.
  
          m_colorTransform->process(m_pFrameStore[2], m_convertFrameStore);
          m_inputTransferFunction->forward(m_pFrameStore[3], m_pFrameStore[2]);
          m_srcDisplayGammaAdjust->forward(m_pFrameStore[3]);
          m_normalizeFunction->forward(m_pFrameStore[1], m_pFrameStore[3]);
  
          m_colorSpaceConvert->process(m_colorSpaceFrame, m_pFrameStore[1]);
          m_outDisplayGammaAdjust->inverse(m_colorSpaceFrame);
          m_oFrameStore->m_colorSpace == CM_ICtCp) {
          m_outputTransferFunction->inverse(m_pFrameStore[6], m_colorSpaceFrame);
          m_colorSpaceConvertMC->process(m_pFrameStore[4], m_pFrameStore[6]);

          // here we apply the output transfer function (to be fixed)
          m_outDisplayGammaAdjust->inverse(m_pFrameStore[1]);
          m_outputTransferFunction->inverse(m_pFrameStore[4],
  
          if (m_iFrameStore->m_chromaFormat != CF_444 &&
              m_oFrameStore->m_chromaFormat != CF_444 &&
              m_iFrameStore->m_colorPrimaries !=
                  m_oFrameStore->m_colorPrimaries) {
              m_convertFormatOut->process(m_pFrameStore[5], m_pFrameStore[4]);
              m_convertProcess->process(m_oFrameStore, m_pFrameStore[5]);
          } else
              m_convertProcess->process(m_oFrameStore, m_pFrameStore[4]);
  
          // 6. frame output
          m_outputFrame->copyFrame(m_oFrameStore);
          m_outputFrame->writeOneFrame(m_outputFile, frameNumber,
                                       m_outputFile->m_fileHeader, 0);
  
      } // end for frameNumber
  }
  ```
