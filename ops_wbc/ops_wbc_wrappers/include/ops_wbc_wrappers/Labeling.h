/*************************************************************************
	> File Name: Labeling.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 15 Jan 2018 04:34:00 PM PST
 ************************************************************************/

#ifndef __OPS_WBC_WRAPPERS_LABELING_H__
#define __OPS_WBC_WRAPPERS_LABELING_H__

#include <iostream>
#include <algorithm>
#include <list>
#include <queue>

#define CLEAR_DST_BUFFER 1
#define CLEAR_ALL_DST_BUFFER 0
#define CALC_CENTER_OF_GRAVITY 1

template<class SrcT, class DstT>
class Labeling
{
    public:
        // raster segment ////////////////////////////////////////////////////
        class RastorSegment
        {
            private:
                int left_x_;
                int right_x_;
                int y_;
                SrcT source_value_;

            public:
                explicit RastorSegment(const int n_lext_x, const int n_right_x,
                                       const int n_y,      const SrcT n_source_value) : left_x_(n_left_x), right_x_(n_right_x), y_(n_y), source_value_(n_source_value)
                {
                }

                ~RastorSegment() = default;

                inline int GetLeftX(void) const 
                {
                    return left_x_;
                }

                inline int GetRightX(void) const 
                {
                    return right_x_;
                }

                inline int GetY(void) const 
                {
                    return y_;
                }

                inline SrcT GetSourceValue(void) const 
                {
                    return source_value_;
                }

                inline int LeftX(void) const
		        {
			        return left_x_;
		        }

		        inline int RightX(void)	const
		        {
			        return right_x_;
		        }
	
		        inline int Y(void) const
		        {
			        return y_;
		        }

		        inline SrcT SourceValue(void) const
		        {
			        return source_value_;
		        }

                friend std::ostream& operator<<(std::ostream& s, RastorSegment& rs)
                {
                    s << rs.LeftX() << " " 
                      << rs.RightX() << " "
                      << rs.Y() << " "
                      << rs.SourceValue() << std::endl;

                    return s;
                }
                
        };

        typedef std::list<RastorSegment *> RSPList;
        typedef typename RSPList::iterator RSPIterator;
        typedef std::queue<RastorSegment *> RSPQueue;

        // information about region //////////////////////////////////////
        class RegionInfo 
        {
            private:
                int num_of_pixels_;
                float center_x_, center_y_;
                int size_x_, size_y_;
                int min_x_, min_y_;
                int max_x_, max_y_;
                SrcT source_value_;
                DstT result_;
                RSPList raster_segment_list_;
#if CALC_CENTER_OF_GRAVITY
                float gravity_x_, gravity_y_;
#endif 
            public:
                // Constructor and Destructor 
                RegionInfo()
                {
                    raster_segment_list_.clear();
                }

                ~RegionInfo()
                {
                    RSPIterator RSPI;
                    for (rspi = raster_segment_list_.begin();
                         rspi != raster_segment_list_.end(); rspi++)
                    {
                        RastorSegment *rs = *rspi;
                        delete rs;
                    }
                    raster_segment_list_.erase(raster_segment_list_.begin(), raster_segment_list_.end());
                }

                inline void SetNumOfPixels(const int n_num_pixels)
                {
                    num_of_pixels_ = n_num_pixels;
                }

                inline void SetCenter(const float x, const float y)
                {
                    center_x_ = x;
                    center_y_ = y;
                }

                inline void SetSize(const int x, const int y)
                {
                    size_x_ = x;
                    size_y_ = y;
                }

                inline void SetMin(const int x, const int y)
		        {
			        min_x_ = x;
			        min_y_ = y;
		        }

		        inline void SetMax(const int x, const int y)
		        {
			        max_x_ = x;
			        max_y_ = y;
		        }

		        inline void SetMinMax(const int n_min_x, const int n_min_y,
				                      const int n_max_x, const int n_max_y)
		        {
			        SetMin(n_min_x, n_min_y);
			        SetMax(n_max_x, n_max_y);
			        SetCenter((n_min_x + n_max_x) / 2.0f,
					          (n_min_y + n_max_y) / 2.0f );
			        SetSize(n_max_x - n_min_x + 1, n_max_y - n_min_y + 1);
		        }

		        inline void SetCenterOfGravity(const float x, const float y)
		        {
			        gravity_x_ = x;
			        gravity_y_ = y;
		        }

		        inline void SetSourceValue(const SrcT n_source_value)
		        {
			        source_value_ = n_source_value;
		        }

		        inline void SetResult(const DstT n_result)
		        {		
			        result_ = n_result;
		        }

                // get

		        inline int GetNumOfPixels(void) const
		        {
			        return num_of_pixels_;
		        }

		        inline void GetCenter(float& x, float& y) const
		        {
			        x = center_x_;
			        y = center_y_;
		        }

		        inline void GetSize(int& x, int& y)	const
		        {
			        x = size_x_;
			        y = size_y_;
		        }

		        inline void GetMin(int& x, int& y) const
		        {
			        x = min_x_;
			        y = min_y_;
		        }

		        inline void GetMax(int& x, int& y) const
		        {
			        x = max_x_;
			        y = max_y_;
		        }

		        inline void GetCenterOfGravity(float& x, float& y) const
		        {
			        x = gravity_x_;
			        y = gravity_y_;
		        }

		        inline SrcT GetSourceValue(void) const
		        {
			        return source_value_;
		        }

		        inline DstT GetResult(void)	const
		        {
			        return result_;
		        }

                // list 
                inline RSPList& GetRasterSegmentList(void)
                {
                    return raster_segment_list_;
                }

                inline void Push(RastorSegment *rs)
                {
                    raster_segment_list_.push_front(rs);
                }

                inline void Pop(RastorSegment *&rs)
                {
                    RSPIterator rspi = raster_segment_list_.begin();
                    rs = *rspi;
                    raster_segment_list_.erase(rspi);
                }

                inline int GetNumRasterSegments(void)
                {
                    return raster_segment_list_.size();
                }

                // operators 
                friend bool operator<(const RegionInfo& l, const RegionInfo& r)
                {
                    bool b = (l.GetNumOfPixels() < r.GetNumOfPixels());
                    return b;
                }

                friend std::ostream& operator<<(std::ostream& s, RegionInfo& ri)
                {
                    int x, y;
                    float cx, cy;

                    s << "num_of_pixels" << ri.GetNumOfPixels() << std::endl;

                    ri.GetCenter(cx, cy);
                    s << "center: " << cx << ", " << cy << std::endl;

                    ri.GetSize(x, y);
			        s << "size: " << x << ", " << y << std::endl;

			        ri.GetMin(x, y);
			        s << "min: " << x << ", " << y << std::endl;

			        ri.GetMax(x, y);
			        s << "max: " << x << ", " << y << std::endl;

#if CALC_CENTER_OF_GRAVITY
			        ri.GetCenterOfGravity(cx, cy);
			        s << "center_of_graivty: " << cx << "," << cy << std::endl;
#endif			

                    s << "source_value: "  
			        << static_cast<int>(ri.GetSourceValue()) << std::endl
			        << "result: "
			        << static_cast<int>(ri.GetResult()) << std::endl;

			        return s;
                }


        };
    
        typedef std::list<RegionInfo *> RIPList;
        typedef RIPList::iterator RIPIterator;
        typedef std::vector<RegionInfo *> RIPVector;


    private:
        static const int DEFAULT_REGION_SIZE_MIN = 10;

        SrcT* src_frame_;
        DstT* dst_frame_;
        int width_;
        int height_;
        int total_num_;

        RSPList* raster_segment_list_;
        int num_of_raster_segments_;
        RSPQueue seed_queue_;
        RIPList region_info_list_;
        int num_of_regions_;
        RIPVector result_region_info_;
        int num_of_result_regions_;

        // private functions 
        void RegisterSegment(const int lx, const int rx,
                             const int y,  const SrcT src_value)
        {
            RasterSegment *rs = new RastorSegment(lx, rx, y, src_value);
            raster_segment_list_[y].push_back(rs);
            num_of_raster_segments_++;
        }

        void SearchNeighboringSegment(RasterSegment* rs_seed, const int dy)
        {
            RSPList *rspl_p = &raster_segment_list_[rs_seed->Y() + dy];
            RSPIterator rspi;

            int rs_seed_lx = rs_seed->LeftX();
            int rs_seed_rx = rs_seed->RightX();
            int rs_seed_source_value = rs_seed->SourceValue();

            rspi = rspl_p->begin();
#if 1
            if (rspi == rspl_p->end())
                return;

            while ((*rspi)->RightX < rs_seed_lx)
            {
                rspi++;
                if (rspi == rspl_p->end())
                {
                    return;
                }
            }

            RastorSegment* rs;
            while ((rs = *rspi)->LeftX <= rs_seed_lx)
            {
                if (rs_seed_source_value == rs->SourceValue())
                {
                    rspi = rspl_p->erase(rspi);
                    seed_queue_.push(rs);
                }
                else
                {
                    rspi++
                }

                if (rspi == rspl_p->end())
                    return;
            }
#endif 
#if 0 
#endif
        }

        RegionInfo* ConnectRasterSegment(RastorSegment* rs_seed, const DstT region_num)
        {
            RegionInfo* ri = new RegionInfo;
            int num_of_pixels = 0;
            int min_x, min_y;
            int max_x, max_y;
            SrcT source_value;

            min_x = rs_seed->LeftX();
            max_x = rs_seed->RightX();
            min_y = max_y = rs_seed->Y();
            source_value = rs_seed->SourceValue();

#if CALC_CENTER_OF_GRAVITY
            int sum_x_ = 0;
            int sum_y_ = 0;
#endif 
            seed_queue_.push(rs_seed);

            while (seed_queue_.size() > 0)
            {
                RastorSegment *rs = seed_queue_.front();
                seed_queue_.pop();
                ri->Push(rs);

                int n = rs->RightX() - rs->LeftX() + 1;
                num_of_pixels += n;

                if (rs->LeftX() < min_x) 
                {
				    min_x = rs->LeftX();
			    }
			    if ( rs->RightX() > max_x ) 
                {
			        max_x = rs->RightX();
			    }

                if (rs->Y() < min_y) 
                {
				    min_y = rs->Y();
			    }
                else if (rs->Y() > max_y) 
                {
				    max_y = rs->Y();
			    }

#if CALC_CENTER_OF_GRAVITY
                sum_x += (rs->LeftX() + rs->RightX()) * n;
                sum_y += rs->Y() * n
#endif 
                if (rs->Y() > 0)
                    SearchNeighboringSegment(rs, -1);

                if (rs->Y() < height_ - 1)
                    SearchNeighboringSegment(rs, 1);

                ri->SetNumOfPixels(num_of_pixels);
                ri->SetMinMax(min_x, min_y, max_x, max_y);
                ri->SetSourceValue(source_value);
                ri->SetResult(region_num);
#if CALC_CENTER_OF_GRAVITY
                float gx = static_cast<float>(sum_x) / (2 * num_of_pixels);
                float gy = static_cast<float>(sum_y) / num_of_pixels;
                ri->SetCenterOfGravity(gx, gy);
#endif 
                return ri;
            }
        }

        static bool RevCompRegionInfoPointer(const RegionInfo * const &l, 
                                             const RegionInfo * const &r)
        {
            bool b = (l->GetNumOfPixels() > r->GetNumOfPixels());
            if (l->GetNumOfPixels() == r->GetNumOfPixels())
            {
                int lx, ly, rx, ry;
                l->GetMin(lx, ly);
                r->GetMin(rx, ry);
                b = (ly > ry);
            }
            return b;
        }

        void FillFrame(RegionInfo *ri, const DstT fill_value)
        {
#if 0
#endif 
            while (ri->GetNumRasterSegments() > 0)
            {
#if 0
                RasterSegment* rs;
                rs->Pop(rs);
                DstT *sp = dst_frame_ + rs->LeftX() + rs->Y() * width;
                for (uint32_t i = 0; i < rs->RightX() - rs->LeftX() + 1; i++)
                {
                    *sp++ = fill_value;
                }
            }
#endif
            RSPList rspl = ri->GetRasterSegmentList();
            for (RSPIterator rspi = rspl.begin(); rspi != rspl.end(); rspi++)
            {
                RastorSegment* rs = *rspi;
                int lx = rs->LeftX();
                int rx = rs->RightX();
                int y = rs->Y();
                DstT *sp = dst_frame_ + lx + y * width;
                for (int i = 0; i < (rx - lx + 1); i++)
                {
                    *sp++ = fill_value;
                }
            }
        }

    public:
        inline int GetNumOfRegions(void) const 
            return num_of_regions_;

        inline int GetNumOfResultRegions(void) const 
            return num_of_result_regions_;

        inline RegionInfo* GetResultRegionInfo(const int num) const 
            return result_region_info_[num]

        Labeling()
        {
            raster_segment_list_ = 0;
            region_info_list_.clear();
            result_region_info_.clear();
        }

        virtual ~Labeling()
        {
            for (RIPIterator ripi = region_info_list_.begin(); ripi != region_info_list_.end(); ripi++)
            {
                RegionInfo *ri = *ripi;
                delete ri;
            }
            region_info_list_.erase(region_info_list_.begin(), region_info_list_.end());
            region_info_list_.clear();
        }
#define CHECK_FOR_PHASE1 0
#define CHECK_FOR_PHASE2 0
        
        int Exec(SrcT *target, DstT *result,
                 int target_width, int target_height,
                 const bool is_sort_region,
                 const int region_size_min)
        {
            src_frame_ = target;
            dst_frame_ = result;

            width_ = target_width;
            height_ = target_height;
            total_num_ = width_ * height_;

            // phase pre1 
            for (RIPIterator ripi = region_info_list_.begin(); ripi != region_info_list_.end(); ripi++)
            {
                RegionInfo *ri = *ripi;
                delete ri;
            }
            region_info_list_.erase(region_info_list_.begin(), region_info_list_.end());
            region_info_list_.clear();
            raster_segment_list_ = new RSPList[height_];
            num_of_raster_segments_ = 0;

            // phase1 
            SrcT *p = src_frame_;

#if (CLEAR_DST_BUFFER || CLEAR_ALL_DST_BUFFER)
            DstT *q = dst_frame_;
#endif 
            if (src_frame_ != reinterpret_cast<SrcT *>(dst_frame_))
            {
#if CLEAR_ALL_DST_BUFFER
                for (int i = 0; i < width_ * height_; i++)
                    *q++ = 0;

                for (int y = 0; y < height_; y++)
                {
                    int lx = 0;
                    int current_src_value = 0;
                    for (int x = 0; x < width_; x++)
                    {
                        if (*p != current_src_value)
                        {
                            if (current_src_value != 0)
                                RegisterSegment(lx, x - 1, y, current_src_value);
                            current_src_value = *p;
                            lx = x;
                        }

#if (CLEAR_DST_BUFFER && !CLEAR_ALL_DST_BUFFER)
                        if (*p = 0)
                            *q = 0;
                        q++;
#endif
                        p++;
                    }
                        
                    if (current_src_value != 0)
                        RegisterSegment(lx, width_ - 1, y, current_src_value);
                }
            }
            else 
            {
                // no need to clear dst_frame if src_frame = dst_frame 
                for (int y = 0; y < height_; y++)
                {
                    int lx = 0;
                    int current_src_value = 0;
                    for (int x = 0; x < width_; x++)
                    {
                        if (*p != current_src_value)
                        {
                            if (current_src_value != 0)
                                RegisterSegment(lx, x - 1, y , current_src_value);
                            current_src_value = *p;
                            lx = x;
                        }
                        p++;
                    }
                    if (current_src_value != 0)
                        RegisterSegment(lx, width_ - 1, y, current_src_value);
                }
            }

#if CHECK_FOR_PHASE1
            for (int y = 0; y < height_; y++)
            {
                cout << y << ":" << raster_segment_list_[y].size() << endl;
                RSPList *rspl_p = &raster_segment_list_[y];
                RSPIterator i;

                for (i = rspl_p->begin(); i != rspl_p->end(); i++)
                {
                    RasterSegment *rs = *i;
                    cout << *rs;
                }
            }
            cout << "num_of_raster_segments" << num_of_raster_segments_ << endl;
#endif 

            // phase pre2 
            region_info_list_.clear();
            num_of_regions_ = 0;

            // phase 2: connect 
            for (int y = 0; y < height_; y++)
            {
                RSPList *rspl_p = &raster_segment_list_[y];
                while (rspl_p->size() > 0)
                {
                    RSPIterator rspi = rspl_p->begin();
                    RasterSegment *rs = *ripi; // get 1 raster segment 
                    rspl_p->erase(rspi);

                    RegionInfo *rip = ConnectRasterSegment(rs, num_of_regions_ + 1);

                    region_info_list_.push_back(rip);
                    num_of_regions_++;
                }
            }
#if CHECK_FOR_PHASE2
            for (int y = 0; y < height; y++) 
            {
			    if (!raster_segment_list[y].empty()) 
                {
				    cout << "mmmm" << y << endl;
			    }
		    }

		    int	n_p = 0;
		    for (RIPIterator ripi = region_info_list.begin(); ripi != region_info_list.end(); ripi++) 
            {
			    RegionInfo	*ri = *ripi;
			    n_p += ri->GetNumOfPixels();
			    while (ri->GetNumOfRasterSegments() > 0) 
                {
				    RasterSegment *rs;
				    ri->Pop(rs);
				    cout << *rs;
			    }
		    }
		    cout << "num_of_pixels: " << n_p << endl;
		    cout << "num_of_regions: " << num_of_regions << endl;
#endif 
            // phase 3 
            // recorder by size 
            result_region_info_.resize(num_of_regions_);
            int n = 0;
            for (RIPIterator ripi = region_info_list_.begin(); ripi != region_info_list_.end(); ripi++)
            {
                result_region_info_[n] = *ripi;
                n++;
            }

            if (is_sort_region)
                sort(result_region_info_.begin(), result_region_info_.end(), RevCompRegionInfoPointer);

            // renumber IDs of RegionInfo
            if (is_sort_region && region_size_min > 0)
            {
                int n = 0;
                while (n < num_of_regions_ && result_region_info_[n]->GetNumOfPixels() >= region_size_min)
                {
                    result_region_info_[n]->SetResult(n+1);
                    n++;
                }
                num_of_result_regions = n;
			    for (int i = n; i < num_of_regions; i++) 
                {
				    result_region_info[ i ]->SetResult( 0 );
			    }
		    } 
            else 
            {
			    for ( int i = 0; i < num_of_regions; i++ ) 
                    result_region_info[ i ]->SetResult( i + 1 );
			    num_of_result_regions = num_of_regions;
            }

            // phase 4 
            // put label number of pixels 
            for (int i = 0; i < num_of_regions_; i++)
            {
                RegionInfo *ri = result_region_info_[i];
                FillFrame(ri, ri->GetResult());
            }

            // clear 
            delete [] raster_segment_list_;
            return 0;
        }
};
typedef Labeling<unsigned char, short> LabelingBS;
typedef Labeling<short, short> LabelingBS;
typedef Labeling,unsigned char, short>::RegionInfo RegionInfoBS;
typedef Labeling<short,short>::RegionInfo RegionInfoSS;
#endif // __OPS_WBC_WRAPPERS_LABELING_H__
